#!/usr/bin/env python3
"""
MPPI 轨迹跟踪模块 (Nav2 风格改进版)
====================================
参考 Nav2 MPPI Controller 架构设计

改进点:
    1. 模块化 Critics 架构 (PathFollowCritic, ObstacleCritic, etc.)
    2. Costmap 查表代替障碍物点列表
    3. 软约束代替硬约束
    4. 路径对齐代价 (Path Align)

依赖: torch, pytorch_mppi

作者: GitHub Copilot & User
"""

import torch
import numpy as np
import rospy

try:
    from pytorch_mppi import MPPI
except ImportError:
    rospy.logerr("pytorch_mppi not found. Please install: pip install pytorch-mppi")
    raise


class MPPITracker:
    """
    MPPI 轨迹跟踪器 (Nav2 风格)

    特点:
        - 模块化 Critics 代价函数
        - 支持 Costmap 查表
        - 软约束设计
        - 控制平滑性惩罚 (防止抖动)
        - 死区设计 (避免过度微调)
    """

    def __init__(self, dt=0.1, N=30):
        """
        初始化 MPPI 跟踪器

        Args:
            dt: 控制周期 (s)
            N: 预测步数 (MPPI horizon)
        """
        self.dt = dt
        self.N = N

        # === 硬件配置 ===
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[MPPITracker] Running on: {self.device}")

        # === 控制约束 ===
        self.v_max = 0.22
        self.v_min = -0.05  # 允许微量倒车
        self.w_max = 2.0

        # === 参考轨迹 Tensor ===
        self.ref_traj_tensor = torch.zeros((self.N, 4), device=self.device, dtype=torch.float32)

        # === Costmap (可选) ===
        self.costmap = None # None 表示不使用 Costmap ，Tensor 表示使用
        self.costmap_resolution = 0.05
        self.costmap_origin = (0.0, 0.0)
        self.costmap_size = (0, 0)

        # === 障碍物点列表 (备用) ===
        self.obstacle_points = None

        # === 上一帧控制量 (用于平滑性惩罚) ===
        # 初始化为 None，第一帧不计算平滑性惩罚
        self.last_action = None

        # === 死区参数 ===
        self.position_deadband = rospy.get_param("~position_deadband", 0.05)  # 5cm
        self.angle_deadband = rospy.get_param("~angle_deadband", 0.087)  # 5度 ≈ 0.087 rad

        # === Critics 权重 (可调参数) ===
        self.weights = {
            'path_follow': rospy.get_param("~w_path_follow", 60.0),   # 路径跟随 (提高)
            'path_align': rospy.get_param("~w_path_align", 40.0),     # 朝向对齐
            'goal_dist': rospy.get_param("~w_goal_dist", 30.0),       # 目标距离 (提高)
            'velocity': rospy.get_param("~w_velocity", 5.0),          # 速度跟踪
            'angular': rospy.get_param("~w_angular", 5.0),            # 角速度惩罚 (降低，让它能转)
            'obstacle': rospy.get_param("~w_obstacle", 100.0),        # 障碑物
            'constraint': rospy.get_param("~w_constraint", 500.0),    # 约束违反
            'smoothness': rospy.get_param("~w_smoothness", 50.0),     # 控制平滑性 (降低，之前150太高)
        }

        # === APF 参数 (障碍物斥力) ===
        self.obs_d0 = rospy.get_param("~obs_d0", 0.4)  # 影响范围
        self.obs_eta = rospy.get_param("~obs_eta", 1.0)  # 斥力强度

        rospy.loginfo(f"[MPPITracker] Critics weights: {self.weights}")
        rospy.loginfo(f"[MPPITracker] Deadband: pos={self.position_deadband}m, ang={np.degrees(self.angle_deadband):.1f}deg")

        # === MPPI 噪声设置 (增大角速度噪声，让它能探索更大的转向) ===
        noise_sigma = torch.tensor([
            [0.1, 0.0],   # 速度噪声
            [0.0, 0.8]    # 角速度噪声
        ], device=self.device, dtype=torch.float32)

        # === 初始化 MPPI 求解器 (软约束模式) ===
        self.mppi = MPPI(
            self._dynamics,
            self._running_cost,
            nx=3,
            noise_sigma=noise_sigma,
            num_samples=800,    # 减少采样数量以加快计算
            horizon=self.N,
            device=self.device,
            lambda_=0.01,
            # 软约束: 扩大范围，让优化器学会约束
            u_min=torch.tensor([self.v_min - 0.1, -self.w_max - 0.5], device=self.device, dtype=torch.float32),
            u_max=torch.tensor([self.v_max + 0.1, self.w_max + 0.5], device=self.device, dtype=torch.float32)
        )

        rospy.loginfo(f"[MPPITracker] Initialized: N={N}, dt={dt}")

    # ==================== 动力学模型 ====================

    def _dynamics(self, state, action):
        """
        差速机器人动力学 (Batch Parallel)

        state: (K, 3) -> x, y, theta
        action: (K, 2) -> v, w
        """
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]

        v = action[:, 0]
        w = action[:, 1]

        new_x = x + v * torch.cos(theta) * self.dt
        new_y = y + v * torch.sin(theta) * self.dt
        new_theta = theta + w * self.dt

        return torch.stack((new_x, new_y, new_theta), dim=1)

    # ==================== 模块化 Critics ====================

    def _running_cost(self, state, action):
        """
        总代价函数 = 各 Critics 加权和

        参考 Nav2 MPPI 的模块化设计
        
        改进:
            1. 添加控制平滑性惩罚 (防止抖动)
            2. 死区设计 (避免过度微调)
        """
        cost = torch.zeros(state.shape[0], device=self.device)

        # 1. PathFollowCritic: 横向偏差 (Cross Track Error) - 带死区
        path_cost = self._path_follow_critic(state)
        # 死区: 如果误差 < position_deadband，代价减小到接近 0
        path_cost = torch.where(
            path_cost < self.position_deadband ** 2,
            path_cost * 0.1,  # 在死区内只保留 10% 的代价
            path_cost
        )
        cost += self.weights['path_follow'] * path_cost

        # 2. PathAlignCritic: 朝向对齐 - 带死区
        align_cost = self._path_align_critic(state)
        # 死区: 小角度误差不惩罚 (1 - cos(5deg) ≈ 0.004)
        align_deadband = 1.0 - np.cos(self.angle_deadband)
        align_cost = torch.where(
            align_cost < align_deadband * 2.0,
            align_cost * 0.1,
            align_cost
        )
        cost += self.weights['path_align'] * align_cost

        # 3. GoalDistCritic: 目标距离
        cost += self.weights['goal_dist'] * self._goal_dist_critic(state)

        # 4. VelocityCritic: 速度跟踪
        cost += self.weights['velocity'] * self._velocity_critic(state, action)

        # 5. AngularCritic: 角速度惩罚
        cost += self.weights['angular'] * self._angular_critic(action)

        # 6. ObstacleCritic: 障碍物避障
        cost += self.weights['obstacle'] * self._obstacle_critic(state)

        # 7. ConstraintCritic: 软约束违反惩罚
        cost += self.weights['constraint'] * self._constraint_critic(action)

        # 8. SmoothnessCritic: 控制平滑性惩罚 (新增)
        cost += self.weights['smoothness'] * self._smoothness_critic(action)

        return cost

    def _path_follow_critic(self, state):
        """
        PathFollowCritic: 计算到参考路径的横向偏差

        不是只看一个点，而是看整条参考路径的最近距离
        """
        # state: (K, 3), ref_traj: (N, 4)
        state_pos = state[:, :2].unsqueeze(1)  # (K, 1, 2)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)  # (1, N, 2)

        # 距离矩阵 (K, N)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        min_dist_sq, _ = torch.min(dists_sq, dim=1)

        return min_dist_sq  # 不开根号，梯度更好

    def _path_align_critic(self, state):
        """
        PathAlignCritic: 评价朝向是否对准路径切线方向

        这比单纯跟踪参考 theta 更好，因为它考虑的是"应该朝哪走"
        """
        # 找到最近的参考点
        state_pos = state[:, :2].unsqueeze(1)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        _, min_indices = torch.min(dists_sq, dim=1)

        # 获取对应的参考 theta
        ref_theta = self.ref_traj_tensor[min_indices, 2]
        robot_theta = state[:, 2]

        # 角度差异代价 (1 - cos(delta_theta))
        delta_theta = torch.atan2(
            torch.sin(robot_theta - ref_theta),
            torch.cos(robot_theta - ref_theta)
        )
        return (1.0 - torch.cos(delta_theta)) * 2.0

    def _goal_dist_critic(self, state):
        """
        GoalDistCritic: 鼓励接近路径终点

        终点是参考轨迹的最后一个点
        """
        goal = self.ref_traj_tensor[-1, :2]  # (2,)
        dist_sq = (state[:, 0] - goal[0])**2 + (state[:, 1] - goal[1])**2
        return dist_sq

    def _velocity_critic(self, state, action):
        """
        VelocityCritic: 速度跟踪代价

        鼓励机器人保持参考速度
        """
        v = action[:, 0]

        # 找最近参考点的速度
        state_pos = state[:, :2].unsqueeze(1)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        _, min_indices = torch.min(dists_sq, dim=1)

        ref_v = self.ref_traj_tensor[min_indices, 3]

        return (v - ref_v)**2

    def _angular_critic(self, action):
        """
        AngularCritic: 惩罚过大的角速度

        防止机器人剧烈转向
        """
        w = action[:, 1]
        return w**2

    def _obstacle_critic(self, state):
        """
        ObstacleCritic: 障碍物避障

        优先使用 Costmap 查表，否则使用点列表 APF
        """
        if self.costmap is not None:
            return self._obstacle_cost_from_costmap(state)
        elif self.obstacle_points is not None and len(self.obstacle_points) > 0:
            return self._obstacle_cost_from_points(state)
        else:
            return torch.zeros(state.shape[0], device=self.device)

    def _obstacle_cost_from_costmap(self, state):
        """
        Costmap 查表法 (Nav2 风格)

        直接查询代价地图值，天然带有梯度
        """
        x = state[:, 0]
        y = state[:, 1]

        # 世界坐标 -> 栅格坐标
        gx = ((x - self.costmap_origin[0]) / self.costmap_resolution).long()
        gy = ((y - self.costmap_origin[1]) / self.costmap_resolution).long()

        # 边界检查
        valid = (gx >= 0) & (gx < self.costmap_size[0]) & \
                (gy >= 0) & (gy < self.costmap_size[1])

        cost = torch.zeros_like(x)

        if valid.any():
            gx_valid = torch.clamp(gx, 0, self.costmap_size[0] - 1)
            gy_valid = torch.clamp(gy, 0, self.costmap_size[1] - 1)

            # 查表
            costmap_vals = self.costmap[gx_valid, gy_valid]

            # 归一化到 [0, 1]
            cost = costmap_vals.float() / 100.0

            # 致命障碍物 (值 > 90) 给极高代价
            lethal = costmap_vals > 90
            cost = torch.where(lethal, torch.tensor(100.0, device=self.device), cost)

        return cost

    def _obstacle_cost_from_points(self, state):
        """
        点列表 APF 法 (备用)

        当没有 Costmap 时使用
        """
        state_pos = state[:, :2].unsqueeze(1)  # (K, 1, 2)
        obs_pos = self.obstacle_points.unsqueeze(0)  # (1, M, 2)

        # 距离 (K, M)
        dists_sq = torch.sum((state_pos - obs_pos)**2, dim=2)
        min_dist_sq, _ = torch.min(dists_sq, dim=1)
        min_dist = torch.sqrt(torch.clamp(min_dist_sq, min=1e-6))

        # APF 斥力势
        d0 = self.obs_d0
        eta = self.obs_eta

        inside = min_dist < d0
        inv_d = torch.where(inside, 1.0 / min_dist, torch.zeros_like(min_dist))
        diff = inv_d - 1.0 / d0
        cost = 0.5 * eta * diff * diff

        return torch.where(inside, cost, torch.zeros_like(cost))

    def _constraint_critic(self, action):
        """
        ConstraintCritic: 软约束违反惩罚

        不使用硬约束 clip，而是将违反约束的情况加到代价里
        """
        v = action[:, 0]
        w = action[:, 1]

        cost = torch.zeros_like(v)

        # 速度约束
        cost += torch.relu(v - self.v_max) * 10.0  # 超速惩罚
        cost += torch.relu(self.v_min - v) * 10.0  # 过慢/倒车惩罚

        # 角速度约束
        cost += torch.relu(torch.abs(w) - self.w_max) * 5.0

        return cost

    def _smoothness_critic(self, action):
        """
        SmoothnessCritic: 控制平滑性惩罚 (新增)

        惩罚控制量相对于上一帧的变化，防止抖动
        """
        # 第一帧没有上一帧控制量，不计算平滑性惩罚
        if self.last_action is None:
            return torch.zeros(action.shape[0], device=self.device)
        
        # 角速度变化惩罚 (主要防止方向抖动)
        w = action[:, 1]
        w_change = (w - self.last_action[1]) ** 2
        
        # 线速度变化惩罚 (防止加减速抖动)
        v = action[:, 0]
        v_change = (v - self.last_action[0]) ** 2
        
        # 角速度变化权重更高，因为方向抖动更明显
        return w_change * 2.0 + v_change * 0.5

    # ==================== 主求解函数 ====================

    def solve(self, current_state, reference_window, obstacles_list=None, costmap_data=None):
        """
        执行 MPPI 求解

        Args:
            current_state: [x, y, theta]
            reference_window: 参考轨迹列表 [{'x', 'y', 'theta', 'v'}, ...]
            obstacles_list: [(x, y), ...] 障碍物点列表 (可选)
            costmap_data: dict {'grid', 'resolution', 'origin'} (可选)

        Returns:
            control: [v, w]
            predicted_traj: 预测轨迹
        """
        # 1. 更新参考轨迹
        ref_len = len(reference_window)
        for i in range(self.N):
            idx = min(i, ref_len - 1)
            pt = reference_window[idx]
            self.ref_traj_tensor[i, 0] = pt['x']
            self.ref_traj_tensor[i, 1] = pt['y']
            self.ref_traj_tensor[i, 2] = pt['theta']
            self.ref_traj_tensor[i, 3] = pt.get('v', 0.15)

        # 2. 更新 Costmap (如果提供)
        if costmap_data is not None:
            self._update_costmap(costmap_data)

        # 3. 更新障碍物点列表 (备用)
        if obstacles_list is not None and len(obstacles_list) > 0:
            self.obstacle_points = torch.tensor(
                obstacles_list, device=self.device, dtype=torch.float32
            )
        else:
            self.obstacle_points = None

        # 4. MPPI 求解
        curr_state_tensor = torch.tensor(
            current_state, dtype=torch.float32, device=self.device
        )
        action = self.mppi.command(curr_state_tensor)

        # 5. 应用软约束后的裁剪 (最终输出还是要在物理范围内)
        v = float(torch.clamp(action[0], self.v_min, self.v_max).item())
        w = float(torch.clamp(action[1], -self.w_max, self.w_max).item())

        # 6. 更新上一帧控制量 (用于平滑性惩罚)
        if self.last_action is None:
            self.last_action = torch.zeros(2, device=self.device, dtype=torch.float32)
        self.last_action[0] = v
        self.last_action[1] = w

        # 7. 生成预测轨迹
        pred_traj = self._predict_trajectory(curr_state_tensor)

        return np.array([v, w]), pred_traj

    def _update_costmap(self, costmap_data):
        """更新 Costmap"""
        grid = costmap_data['grid']
        self.costmap_resolution = costmap_data['resolution']
        self.costmap_origin = costmap_data['origin']

        # 转换为 Tensor
        self.costmap = torch.tensor(grid, device=self.device, dtype=torch.float32)
        self.costmap_size = (self.costmap.shape[0], self.costmap.shape[1])

    def _predict_trajectory(self, start_state):
        """预测轨迹 (用于可视化)"""
        traj = []
        state = start_state.clone()
        U = self.mppi.U

        for i in range(min(self.N, U.shape[0])):
            act = U[i]
            next_s = self._dynamics(state.unsqueeze(0), act.unsqueeze(0)).squeeze(0)
            state = next_s
            traj.append([state[0].item(), state[1].item(), state[2].item()])

        return traj

    def reset(self):
        """重置 MPPI 内部状态"""
        self.mppi.reset()
        # 重置为 None，这样第一帧不会有平滑性惩罚
        self.last_action = None

    def set_costmap(self, grid, resolution, origin):
        """
        设置 Costmap (从 GlobalPlanner 获取)

        Args:
            grid: 2D numpy array, 0=free, 1=obstacle
            resolution: 栅格分辨率 (m/cell)
            origin: (x, y) 地图原点
        """
        # 转换: 0=free -> 0, 1=obstacle -> 100
        self.costmap = torch.tensor(grid * 100, device=self.device, dtype=torch.float32)
        self.costmap_resolution = resolution
        self.costmap_origin = origin
        self.costmap_size = (grid.shape[0], grid.shape[1])
        rospy.loginfo(f"[MPPITracker] Costmap set: {self.costmap_size}, res={resolution}")
