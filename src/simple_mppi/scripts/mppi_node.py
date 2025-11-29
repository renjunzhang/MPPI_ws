#!/usr/bin/env python3
"""
MPPI 轨迹跟踪模块 (适配三层架构 - 修复版)
======================================
基于 pytorch_mppi 的轨迹跟踪器

功能:
    - 适配 Layer 2 传来的参考轨迹窗口
    - 使用 GPU 并行采样寻找最优控制量
    - 接口与 MPCTracker 保持一致 (Drop-in Replacement)
    - 修复了数据类型错误和维度索引错误

依赖: torch, pytorch_mppi

作者: GitHub Copilot & User
"""

import torch
import numpy as np
import rospy
from pytorch_mppi import MPPI

class MPPITracker:
    def __init__(self, dt=0.1, N=60):
        """
        初始化 MPPI 跟踪器
        Args:
            dt: 控制周期 (s)
            N: 预测步数 (MPPI 的 horizon)
        """
        self.dt = dt
        self.N = N  # Horizon
        
        # --- 1. 硬件配置 ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[MPPITracker] Running on: {self.device}")
        
        # --- 2. 内部变量 ---
        self.obstacle_points = None # 存储障碍物 Tensor
        
        # 上一次的控制量，用于平滑
        self.last_action = torch.zeros(2, device=self.device, dtype=torch.float32)
        
        # 参考轨迹 Tensor (Horizon, 4) -> [x, y, theta, v]
        # 必须显式指定 float32 防止类型冲突
        self.ref_traj_tensor = torch.zeros((self.N, 4), device=self.device, dtype=torch.float32)
        
        # --- 3. MPPI 参数配置 ---
        self.v_max = 0.22  # TurtleBot3 物理极限
        self.w_max = 2.0
        
        # 噪声设置: [v_sigma, w_sigma]
        # w 的噪声大一点，允许它大幅度尝试转向
        # 必须显式指定 float32
        noise_sigma = torch.tensor([[0.05, 0.0], [0.0, 0.5]], device=self.device, dtype=torch.float32)
        
        # Lambda: 温度系数，越小越贪婪(只选最好的)，越大越平滑(平均)
        self.lambda_ = 0.02 

        # --- 4. APF 参数（吸引 + 斥力）---
        self.use_apf = rospy.get_param("~use_apf", True)

        self.apf_k_att = rospy.get_param("~apf_k_att", 160.0)
        self.apf_eta   = rospy.get_param("~apf_eta", 1.0)
        self.apf_d0    = rospy.get_param("~apf_d0", 0.6)

        rospy.loginfo(f"[MPPITracker] APF enabled={self.use_apf}, "
                      f"k_att={self.apf_k_att}, eta={self.apf_eta}, d0={self.apf_d0}")


        
        # 初始化求解器
        self.mppi = MPPI(self._dynamics, self._running_cost, nx=3, 
                         noise_sigma=noise_sigma,
                         num_samples=1000,    # 采样数 1000
                         horizon=self.N,     # 预测视野
                         device=self.device,
                         lambda_=self.lambda_,
                         u_min=torch.tensor([-0.05, -self.w_max], device=self.device, dtype=torch.float32),
                         u_max=torch.tensor([self.v_max, self.w_max], device=self.device, dtype=torch.float32))
        
        rospy.loginfo("[MPPITracker] Initialized")

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
        
        # 简单的欧拉积分
        new_x = x + v * torch.cos(theta) * self.dt
        new_y = y + v * torch.sin(theta) * self.dt
        new_theta = theta + w * self.dt
        
        return torch.stack((new_x, new_y, new_theta), dim=1)

    def _running_cost(self, state, action):
        """ 
        MPPI + APF 融合的运行代价：
        - 吸引势：靠近参考轨迹（min_dist）
        - 斥力势：远离障碍物（最近障碍距离）
        - 再加上速度/角速度等控制项
        state:  (K, 3)
        action: (K, 2)
        """
        # 1. 拆状态和动作
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]
        
        v = action[:, 0]
        w = action[:, 1]
        
        # 2. 计算到参考轨迹的最近距离（吸引势的“目标”）
        # state_pos: (K, 1, 2)
        state_pos = state[:, :2].unsqueeze(1)
        # ref_pos: (1, N, 2)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        
        # 距离矩阵 (K, N)
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2)
        # 最近点索引 & 距离
        min_dist_sq, min_indices = torch.min(dists_sq, dim=1)  # (K,)

        # 对应的参考点（包括 theta, v）
        matched_ref = self.ref_traj_tensor[min_indices]  # (K, 4)
        ref_theta = matched_ref[:, 2]
        ref_v = matched_ref[:, 3]
        
        # 3. APF 吸引势：偏离轨迹的惩罚
        #    U_att = 0.5 * k_att * ||p - p_ref||^2
        cost_pos = 0.5 * self.apf_k_att * min_dist_sq
        
        # 4. 姿态误差代价（保持原有形式）
        delta_theta = torch.atan2(
            torch.sin(theta - ref_theta),
            torch.cos(theta - ref_theta)
        )
        cost_angle = (1.0 - torch.cos(delta_theta)) * 80.0
        
        # 5. 速度跟踪 + 控制代价
        cost_v = (v - ref_v) ** 2 * 10.0
        cost_w = (w ** 2) * 0.8
        
        # 6. APF 斥力势：最近障碍物
        cost_obs = torch.zeros_like(cost_pos)
        if self.obstacle_points is not None and len(self.obstacle_points) > 0:
            # obs_pos: (1, M, 2)
            obs_pos = self.obstacle_points.unsqueeze(0)
            # 每个采样点到所有障碍物的距离平方 (K, M)
            obs_dists_sq = torch.sum((state_pos - obs_pos) ** 2, dim=2)
            # 最近障碍距离
            min_obs_dist_sq, _ = torch.min(obs_dists_sq, dim=1)
            # 防止 sqrt(0)
            min_obs_dist = torch.sqrt(torch.clamp(min_obs_dist_sq, min=1e-6))
            
            d0 = self.apf_d0   # 影响范围
            eta = self.apf_eta # 斥力强度
            
            # 只在 d <= d0 时产生斥力势
            inside = min_obs_dist < d0  # (K,)
            
            # 1/d - 1/d0，数值稳定起见用 where
            inv_d = torch.where(
                inside,
                1.0 / min_obs_dist,
                torch.zeros_like(min_obs_dist)
            )
            diff = inv_d - 1.0 / d0
            U_rep = 0.5 * eta * diff * diff  # (K,)
            
            cost_obs = torch.where(inside, U_rep, torch.zeros_like(U_rep))
        
        total_cost = cost_pos + cost_angle + cost_v + cost_w

        if self.use_apf:
            total_cost = total_cost + cost_obs   # cost_obs 就是斥力势

        return total_cost

    


    def solve(self, current_state, reference_window, obstacles_list=None):
        """
        执行 MPPI 求解
        Args:
            current_state: [x, y, theta]
            reference_window: 参考轨迹列表 (Layer 2 Output)
            obstacles_list: [(x, y), ...] 障碍物列表 (Global input)
        Returns:
            control: [v, w]
            predicted_traj: 预测轨迹用于可视化
        """
        # 1. 更新参考轨迹 Tensor
        ref_len = len(reference_window)
        for i in range(self.N):
            idx = min(i, ref_len - 1)
            pt = reference_window[idx]
            
            self.ref_traj_tensor[i, 0] = pt['x']
            self.ref_traj_tensor[i, 1] = pt['y']
            self.ref_traj_tensor[i, 2] = pt['theta']
            # 新增：把速度也存进去！
            self.ref_traj_tensor[i, 3] = pt['v'] 
            
        # 2. 更新障碍物 Tensor
        if obstacles_list is not None and len(obstacles_list) > 0:
            # 必须显式指定 float32
            self.obstacle_points = torch.tensor(obstacles_list, device=self.device, dtype=torch.float32)
        else:
            self.obstacle_points = None

        # 3. 准备状态
        curr_state_tensor = torch.tensor(current_state, dtype=torch.float32, device=self.device)
        
        # 4. MPPI 求解
        action = self.mppi.command(curr_state_tensor)
        
        # 5. 更新内部状态
        self.last_action = action.clone()
        
        # 6. 提取结果
        v = action[0].item()
        w = action[1].item()
        
        # 7. 生成预测轨迹 (用于可视化)
        pred_traj = self._predict_trajectory(curr_state_tensor)
        
        return np.array([v, w]), pred_traj

    def _predict_trajectory(self, start_state):
        """ 使用最优控制序列推演预测轨迹 """
        traj = []
        state = start_state.clone()
        U = self.mppi.U
        
        for i in range(min(self.N, U.shape[0])):
            act = U[i]
            # 扩展维度适配 dynamics (Batch=1)
            next_s = self._dynamics(state.unsqueeze(0), act.unsqueeze(0)).squeeze(0)
            state = next_s
            traj.append([state[0].item(), state[1].item(), state[2].item()])
        return traj

    def reset(self):
        """ 重置 MPPI 内部状态 (比如 U 序列) """
        self.mppi.reset()