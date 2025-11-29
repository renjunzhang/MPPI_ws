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
    def __init__(self, dt=0.1, N=30):
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
        noise_sigma = torch.tensor([[0.08, 0.0], [0.0, 0.8]], device=self.device, dtype=torch.float32)
        
        # Lambda: 温度系数，越小越贪婪(只选最好的)，越大越平滑(平均)
        self.lambda_ = 0.02 
        
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
        修正版代价函数：适配 (K, 3) 输入维度，修复索引错误
        """
        # --- 1. 维度修正 ---
        # state 现在的形状是 (K, 3)，不是 (K, N, 3)
        # action 现在的形状是 (K, 2)
        
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]
        
        v = action[:, 0]
        w = action[:, 1]
        
        # --- 2. 计算与参考轨迹的最近距离 (Path Following) ---
        # state_pos: (K, 1, 2)
        state_pos = state[:, :2].unsqueeze(1)
        # ref_pos: (1, N, 2)
        ref_pos = self.ref_traj_tensor[:, :2].unsqueeze(0)
        
        # 计算距离矩阵 (K, N) -> 每个采样点到所有参考点的距离
        # 这一步利用广播机制，计算量稍大但 GPU 扛得住
        dists_sq = torch.sum((state_pos - ref_pos)**2, dim=2) # (K, N)
        
        # 找到最近点的索引和距离
        min_dist_sq, min_indices = torch.min(dists_sq, dim=1) # (K,)
        
        # --- 3. 提取匹配点的参考信息 ---
        # 根据最近点的索引，找出对应的参考速度和角度
        # self.ref_traj_tensor 是 (N, 4) -> [x, y, theta, v]
        
        # 使用索引直接提取匹配的参考点
        matched_ref = self.ref_traj_tensor[min_indices] # (K, 4)
        
        ref_theta = matched_ref[:, 2]
        ref_v = matched_ref[:, 3]
        
        # --- 4. 计算各项代价 ---
        
        # A. 位置代价 (Cross Track Error)
        cost_pos = min_dist_sq * 60.0
        
        # B. 朝向代价 (Angle Error - 修复绕圈问题)
        # 使用 1-cos 处理角度跳变
        delta_theta = theta - ref_theta
        cost_angle = (1.0 - torch.cos(delta_theta)) * 80.0
        
        # C. 速度代价 (Tracking Speed)
        cost_v = (v - ref_v)**2 * 10.0
        
        # D. 动作平滑与约束
        cost_w = (w**2) * 0.8
        
        # E. 障碍物代价 (如果有)
        cost_obs = torch.zeros_like(cost_pos)
        if self.obstacle_points is not None and len(self.obstacle_points) > 0:
            obs_pos = self.obstacle_points.unsqueeze(0) # (1, M, 2)
            # 计算到所有障碍物的距离
            obs_dists_sq = torch.sum((state_pos - obs_pos)**2, dim=2) # (K, M)
            # 最近障碍物
            min_obs_dist_sq, _ = torch.min(obs_dists_sq, dim=1)
            min_obs_dist = torch.sqrt(min_obs_dist_sq)
            
            safe_margin = 0.1
            cost_obs = torch.clamp(safe_margin - min_obs_dist, min=0) * 40.0

        return cost_pos + cost_angle + cost_v + cost_w + cost_obs

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