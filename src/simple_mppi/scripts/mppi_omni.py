#!/usr/bin/env python3
"""
MPPI 轨迹跟踪模块 - 全向移动版本 (Ridgeback)
=============================================
参考 Nav2 MPPI Controller 架构设计

与差速版本的主要区别:
    1. 状态空间: [x, y, theta] (相同)
    2. 控制空间: [vx, vy, omega] (3维, 差速是2维 [v, omega])
    3. 动力学模型: 全向运动学 (可任意方向平移)

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


class MPPIOmniTracker:
    """
    MPPI 轨迹跟踪器 - 全向移动版本

    特点:
        - 3维控制输入 [vx, vy, omega]
        - 全向运动学模型
        - 支持横向运动 (侧移)
        - 模块化 Critics 代价函数
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
        rospy.loginfo(f"[MPPIOmniTracker] Running on: {self.device}")

        # === 控制约束 (Ridgeback 全向平台) ===
        self.vx_max = rospy.get_param("~vx_max", 0.5)
        self.vy_max = rospy.get_param("~vy_max", 0.5)
        self.vx_min = -self.vx_max
        self.vy_min = -self.vy_max
        self.w_max = rospy.get_param("~w_max", 1.0)

        # === 参考轨迹 Tensor ===
        # [x, y, theta, vx, vy] - 全向需要 vx, vy 两个速度分量
        self.ref_traj_tensor = torch.zeros((self.N, 5), device=self.device, dtype=torch.float32)

        # === Costmap (可选) ===
        self.costmap = None
        self.costmap_resolution = 0.05
        self.costmap_origin = (0.0, 0.0)
        self.costmap_size = (0, 0)

        # === 障碍物点列表 (备用) ===
        self.obstacle_points = None

        # === 上一帧控制量 (用于平滑性惩罚) ===
        self.last_action = None

        # === 死区参数 ===
        self.position_deadband = rospy.get_param("~position_deadband", 0.05)
        self.angle_deadband = rospy.get_param("~angle_deadband", 0.087)

        # === Critics 权重 ===
        self.weights = {
            'path_follow': rospy.get_param("~w_path_follow", 50.0),
            'path_align': rospy.get_param("~w_path_align", 20.0),
            'goal_dist': rospy.get_param("~w_goal_dist", 30.0),
            'velocity': rospy.get_param("~w_velocity", 5.0),
            'lateral': rospy.get_param("~w_lateral", 5.0),     # 横向速度惩罚
            'angular': rospy.get_param("~w_angular", 3.0),
            'obstacle': rospy.get_param("~w_obstacle", 100.0),
            'constraint': rospy.get_param("~w_constraint", 500.0),
            'smoothness': rospy.get_param("~w_smoothness", 30.0),
        }

        # === APF 参数 ===
        self.obs_d0 = rospy.get_param("~obs_d0", 0.6)
        self.obs_eta = rospy.get_param("~obs_eta", 1.0)

        rospy.loginfo(f"[MPPIOmniTracker] Critics weights: {self.weights}")
        rospy.loginfo(f"[MPPIOmniTracker] Velocity limits: vx_max={self.vx_max}, vy_max={self.vy_max}, w_max={self.w_max}")

        # === MPPI 噪声设置 (3维控制) ===
        noise_sigma = torch.tensor([
            [0.15, 0.0,  0.0],   # vx 噪声
            [0.0,  0.15, 0.0],   # vy 噪声
            [0.0,  0.0,  0.5]    # omega 噪声
        ], device=self.device, dtype=torch.float32)

        # === 初始化 MPPI 求解器 ===
        self.mppi = MPPI(
            self._dynamics,
            self._running_cost,
            nx=3,  # 状态维度 [x, y, theta]
            noise_sigma=noise_sigma,
            num_samples=800,
            horizon=self.N,
            device=self.device,
            lambda_=0.01,
            # 软约束范围
            u_min=torch.tensor([self.vx_min - 0.1, self.vy_min - 0.1, -self.w_max - 0.3],
                              device=self.device, dtype=torch.float32),
            u_max=torch.tensor([self.vx_max + 0.1, self.vy_max + 0.1, self.w_max + 0.3],
                              device=self.device, dtype=torch.float32)
        )

        rospy.loginfo(f"[MPPIOmniTracker] Initialized: N={N}, dt={dt}, nu=3 (omnidirectional)")

    # ==================== 动力学模型 ====================

    def _dynamics(self, state, action):
        """
        全向机器人动力学 (Batch Parallel)

        与差速的区别:
            - 差速: 只能沿 theta 方向前进, v 是沿 theta 的速度
            - 全向: vx, vy 是世界坐标系/机体坐标系的速度分量

        这里使用机体坐标系 (Body Frame) 定义:
            - vx: 机器人前进方向速度
            - vy: 机器人侧向速度 (正值向左)
            - omega: 角速度

        state: (K, 3) -> x, y, theta
        action: (K, 3) -> vx_body, vy_body, omega
        """
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]

        vx_body = action[:, 0]
        vy_body = action[:, 1]
        omega = action[:, 2]

        # 机体坐标系 -> 世界坐标系
        cos_theta = torch.cos(theta)
        sin_theta = torch.sin(theta)

        vx_world = vx_body * cos_theta - vy_body * sin_theta
        vy_world = vx_body * sin_theta + vy_body * cos_theta

        # 更新状态
        new_x = x + vx_world * self.dt
        new_y = y + vy_world * self.dt
        new_theta = theta + omega * self.dt

        return torch.stack((new_x, new_y, new_theta), dim=1)

    # ==================== 模块化 Critics ====================

    def _running_cost(self, state, action):
        """
        总代价函数 = 各 Critics 加权和
        """
        cost = torch.zeros(state.shape[0], device=self.device)

        # 1. PathFollowCritic: 位置偏差
        path_cost = self._path_follow_critic(state)
        path_cost = torch.where(
            path_cost < self.position_deadband ** 2,
            path_cost * 0.1,
            path_cost
        )
        cost += self.weights['path_follow'] * path_cost

        # 2. PathAlignCritic: 朝向对齐
        align_cost = self._path_align_critic(state)
        align_deadband = 1.0 - np.cos(self.angle_deadband)
        align_cost = torch.where(
            align_cost < align_deadband * 2.0,
            align_cost * 0.1,
            align_cost
        )
        cost += self.weights['path_align'] * align_cost

        # 3. GoalDistCritic: 目标距离
        cost += self.weights['goal_dist'] * self._goal_dist_critic(state)

        # 4. VelocityCritic: 前向速度跟踪
        cost += self.weights['velocity'] * self._velocity_critic(state, action)

        # 5. LateralCritic: 横向速度惩罚 (鼓励正向运动)
        cost += self.weights['lateral'] * self._lateral_critic(action)

        # 6. AngularCritic: 角速度惩罚
        cost += self.weights['angular'] * self._angular_critic(action)

        # 7. ObstacleCritic: 障碍物避障
        cost += self.weights['obstacle'] * self._obstacle_critic(state)

        # 8. ConstraintCritic: 软约束
        cost += self.weights['constraint'] * self._constraint_critic(action)

        # 9. SmoothnessCritic: 平滑性
        cost += self.weights['smoothness'] * self._smoothness_critic(action)

        return cost

    def _path_follow_critic(self, state):
        """PathFollowCritic: 到参考路径的最近距离"""
        state_pos = state[:, :2].unsqueeze(1)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        min_dist_sq, _ = torch.min(dists_sq, dim=1)
        return min_dist_sq

    def _path_align_critic(self, state):
        """PathAlignCritic: 朝向对齐"""
        state_pos = state[:, :2].unsqueeze(1)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        _, min_indices = torch.min(dists_sq, dim=1)

        ref_theta = self.ref_traj_tensor[min_indices, 2]
        robot_theta = state[:, 2]

        delta_theta = torch.atan2(
            torch.sin(robot_theta - ref_theta),
            torch.cos(robot_theta - ref_theta)
        )
        return (1.0 - torch.cos(delta_theta)) * 2.0

    def _goal_dist_critic(self, state):
        """GoalDistCritic: 目标距离"""
        goal = self.ref_traj_tensor[-1, :2]
        dist_sq = (state[:, 0] - goal[0])**2 + (state[:, 1] - goal[1])**2
        return dist_sq

    def _velocity_critic(self, state, action):
        """VelocityCritic: 前向速度跟踪"""
        vx = action[:, 0]

        state_pos = state[:, :2].unsqueeze(1)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        _, min_indices = torch.min(dists_sq, dim=1)

        # 参考速度 (前向速度分量)
        ref_vx = self.ref_traj_tensor[min_indices, 3]

        return (vx - ref_vx)**2

    def _lateral_critic(self, action):
        """
        LateralCritic: 横向速度惩罚

        虽然全向平台可以侧移，但通常希望机器人主要向前运动，
        只在必要时使用侧移。轻微惩罚横向速度。
        """
        vy = action[:, 1]
        return vy**2

    def _angular_critic(self, action):
        """AngularCritic: 角速度惩罚"""
        omega = action[:, 2]
        return omega**2

    def _obstacle_critic(self, state):
        """ObstacleCritic: 障碍物避障"""
        if self.costmap is not None:
            return self._obstacle_cost_from_costmap(state)
        elif self.obstacle_points is not None and len(self.obstacle_points) > 0:
            return self._obstacle_cost_from_points(state)
        else:
            return torch.zeros(state.shape[0], device=self.device)

    def _obstacle_cost_from_costmap(self, state):
        """Costmap 查表"""
        x = state[:, 0]
        y = state[:, 1]

        gx = ((x - self.costmap_origin[0]) / self.costmap_resolution).long()
        gy = ((y - self.costmap_origin[1]) / self.costmap_resolution).long()

        valid = (gx >= 0) & (gx < self.costmap_size[0]) & \
                (gy >= 0) & (gy < self.costmap_size[1])

        cost = torch.zeros_like(x)

        if valid.any():
            gx_valid = torch.clamp(gx, 0, self.costmap_size[0] - 1)
            gy_valid = torch.clamp(gy, 0, self.costmap_size[1] - 1)
            costmap_vals = self.costmap[gx_valid, gy_valid]
            cost = costmap_vals.float() / 100.0
            lethal = costmap_vals > 90
            cost = torch.where(lethal, torch.tensor(100.0, device=self.device), cost)

        return cost

    def _obstacle_cost_from_points(self, state):
        """APF 障碍物避障"""
        state_pos = state[:, :2].unsqueeze(1)
        obs_pos = self.obstacle_points.unsqueeze(0)

        dists_sq = torch.sum((state_pos - obs_pos)**2, dim=2)
        min_dist_sq, _ = torch.min(dists_sq, dim=1)
        min_dist = torch.sqrt(torch.clamp(min_dist_sq, min=1e-6))

        d0 = self.obs_d0
        eta = self.obs_eta

        inside = min_dist < d0
        inv_d = torch.where(inside, 1.0 / min_dist, torch.zeros_like(min_dist))
        diff = inv_d - 1.0 / d0
        cost = 0.5 * eta * diff * diff

        return torch.where(inside, cost, torch.zeros_like(cost))

    def _constraint_critic(self, action):
        """ConstraintCritic: 软约束"""
        vx = action[:, 0]
        vy = action[:, 1]
        omega = action[:, 2]

        cost = torch.zeros_like(vx)

        # 速度约束
        cost += torch.relu(vx - self.vx_max) * 10.0
        cost += torch.relu(self.vx_min - vx) * 10.0
        cost += torch.relu(vy - self.vy_max) * 10.0
        cost += torch.relu(self.vy_min - vy) * 10.0

        # 角速度约束
        cost += torch.relu(torch.abs(omega) - self.w_max) * 5.0

        return cost

    def _smoothness_critic(self, action):
        """SmoothnessCritic: 控制平滑性"""
        if self.last_action is None:
            return torch.zeros(action.shape[0], device=self.device)

        vx_change = (action[:, 0] - self.last_action[0]) ** 2
        vy_change = (action[:, 1] - self.last_action[1]) ** 2
        omega_change = (action[:, 2] - self.last_action[2]) ** 2

        return vx_change * 0.5 + vy_change * 0.5 + omega_change * 2.0

    # ==================== 主求解函数 ====================

    def solve(self, current_state, reference_window, obstacles_list=None, costmap_data=None):
        """
        执行 MPPI 求解

        Args:
            current_state: [x, y, theta]
            reference_window: 参考轨迹 [{'x', 'y', 'theta', 'vx', 'vy'}, ...]
            obstacles_list: [(x, y), ...]
            costmap_data: dict {'grid', 'resolution', 'origin'}

        Returns:
            control: [vx, vy, omega]
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
            self.ref_traj_tensor[i, 3] = pt.get('vx', pt.get('v', 0.2))  # 兼容旧格式
            self.ref_traj_tensor[i, 4] = pt.get('vy', 0.0)

        # 2. 更新 Costmap
        if costmap_data is not None:
            self._update_costmap(costmap_data)

        # 3. 更新障碍物点列表
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

        # 5. 裁剪到物理限制
        vx = float(torch.clamp(action[0], self.vx_min, self.vx_max).item())
        vy = float(torch.clamp(action[1], self.vy_min, self.vy_max).item())
        omega = float(torch.clamp(action[2], -self.w_max, self.w_max).item())

        # 6. 更新上一帧控制量
        if self.last_action is None:
            self.last_action = torch.zeros(3, device=self.device, dtype=torch.float32)
        self.last_action[0] = vx
        self.last_action[1] = vy
        self.last_action[2] = omega

        # 7. 预测轨迹
        pred_traj = self._predict_trajectory(curr_state_tensor)

        return np.array([vx, vy, omega]), pred_traj

    def _update_costmap(self, costmap_data):
        """更新 Costmap"""
        grid = costmap_data['grid']
        self.costmap_resolution = costmap_data['resolution']
        self.costmap_origin = costmap_data['origin']
        self.costmap = torch.tensor(grid, device=self.device, dtype=torch.float32)
        self.costmap_size = (self.costmap.shape[0], self.costmap.shape[1])

    def _predict_trajectory(self, start_state):
        """预测轨迹"""
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
        """重置 MPPI"""
        self.mppi.reset()
        self.last_action = None

    def set_costmap(self, grid, resolution, origin):
        """设置 Costmap"""
        self.costmap = torch.tensor(grid * 100, device=self.device, dtype=torch.float32)
        self.costmap_resolution = resolution
        self.costmap_origin = origin
        self.costmap_size = (grid.shape[0], grid.shape[1])
        rospy.loginfo(f"[MPPIOmniTracker] Costmap set: {self.costmap_size}, res={resolution}")
