#!/usr/bin/env python3
"""
MPC 轨迹跟踪控制模块 (Layer 3)
==============================
基于模型预测控制的轨迹跟踪

功能:
    - 差速驱动机器人运动学模型
    - 非线性 MPC 优化
    - 参考轨迹跟踪
    - 控制平滑约束

输入: 当前状态 [x, y, theta], 参考轨迹窗口
输出: 控制量 [v, omega], 预测轨迹

依赖: CasADi, IPOPT

作者: GitHub Copilot
"""

import numpy as np
import casadi as ca

# ROS 依赖 (可选)
try:
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


def _log_info(msg):
    if HAS_ROS and not rospy.is_shutdown():
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")


def _log_warn(msg):
    if HAS_ROS and not rospy.is_shutdown():
        rospy.logwarn(msg)
    else:
        print(f"[WARN] {msg}")


class MPCTracker:
    """
    MPC 轨迹跟踪控制器
    
    使用非线性模型预测控制跟踪参考轨迹，
    考虑差速驱动机器人的运动学约束。
    
    代价函数:
        - 位置跟踪误差
        - 朝向跟踪误差
        - 速度跟踪误差
        - 控制量大小
        - 控制变化率 (平滑性)
    
    Attributes:
        dt (float): 控制周期 (s)
        N (int): 预测步数
        v_max, v_min (float): 线速度约束 (m/s)
        w_max (float): 角速度约束 (rad/s)
    """
    
    def __init__(self, dt=0.1, N=20,
                 v_max=0.22, v_min=-0.05, w_max=1.8,
                 Q_pos=15.0, Q_theta=5.0, Q_v=3.0,
                 R_v=0.05, R_w=0.05, R_dv=1.0, R_dw=1.5):
        """
        初始化 MPC 控制器
        
        Args:
            dt: 控制周期 (s)
            N: 预测步数
            v_max, v_min: 线速度范围 (m/s)
            w_max: 最大角速度 (rad/s)
            Q_pos: 位置跟踪权重
            Q_theta: 朝向跟踪权重
            Q_v: 速度跟踪权重
            R_v, R_w: 控制量权重
            R_dv, R_dw: 控制变化率权重 (平滑性)
        """
        self.dt = dt
        self.N = N
        
        # 控制约束
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        
        # 代价权重
        self.Q_pos = Q_pos
        self.Q_theta = Q_theta
        self.Q_v = Q_v
        self.R_v = R_v
        self.R_w = R_w
        self.R_dv = R_dv
        self.R_dw = R_dw
        
        # 构建优化器
        self._setup_solver()
        
        # Warm start 变量
        self._init_warm_start()
        
        _log_info(f"[MPCTracker] Initialized: N={N}, dt={dt}")
    
    def _setup_solver(self):
        """构建 CasADi NLP 求解器"""
        
        # === 符号变量 ===
        # 状态: [x, y, theta]
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        state = ca.vertcat(x, y, theta)
        n_state = 3
        
        # 控制: [v, omega]
        v = ca.SX.sym('v')
        w = ca.SX.sym('w')
        control = ca.vertcat(v, w)
        n_control = 2
        
        # 差速驱动运动学
        rhs = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            w
        )
        self.f = ca.Function('f', [state, control], [rhs])
        
        # === 优化变量 ===
        X = ca.SX.sym('X', n_state, self.N + 1)    # 状态序列
        U = ca.SX.sym('U', n_control, self.N)      # 控制序列
        
        # 参数: 初始状态(3) + 参考点((N+1)*4) + 上一次控制(2)
        n_ref_per_point = 4  # [x_ref, y_ref, theta_ref, v_ref]
        P = ca.SX.sym('P', n_state + (self.N + 1) * n_ref_per_point + n_control)
        
        # === 构建代价函数和约束 ===
        obj = 0
        g = []
        
        # 初始状态约束
        g.append(X[:, 0] - P[:n_state])
        
        # 提取上一次控制
        v_last = P[n_state + (self.N + 1) * n_ref_per_point]
        w_last = P[n_state + (self.N + 1) * n_ref_per_point + 1]
        
        for k in range(self.N):
            # 提取参考点
            ref_idx = n_state + k * n_ref_per_point
            x_ref = P[ref_idx]
            y_ref = P[ref_idx + 1]
            theta_ref = P[ref_idx + 2]
            v_ref = P[ref_idx + 3]
            
            # 当前状态和控制
            x_k, y_k, theta_k = X[0, k], X[1, k], X[2, k]
            v_k, w_k = U[0, k], U[1, k]
            
            # 跟踪误差代价
            obj += self.Q_pos * ((x_k - x_ref)**2 + (y_k - y_ref)**2)
            
            # 朝向误差 (考虑周期性)
            theta_err = ca.atan2(ca.sin(theta_k - theta_ref), ca.cos(theta_k - theta_ref))
            obj += self.Q_theta * theta_err**2
            
            # 速度跟踪
            obj += self.Q_v * (v_k - v_ref)**2
            
            # 控制量代价
            obj += self.R_v * v_k**2 + self.R_w * w_k**2
            
            # 控制变化率代价 (平滑性)
            if k == 0:
                dv = v_k - v_last
                dw = w_k - w_last
            else:
                dv = v_k - U[0, k-1]
                dw = w_k - U[1, k-1]
            obj += self.R_dv * dv**2 + self.R_dw * dw**2
            
            # 动力学约束
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])
            g.append(X[:, k+1] - x_next)
        
        # 终端代价 (加权)
        ref_idx = n_state + self.N * n_ref_per_point
        obj += self.Q_pos * 3.0 * ((X[0, self.N] - P[ref_idx])**2 + 
                                   (X[1, self.N] - P[ref_idx + 1])**2)
        theta_err_N = ca.atan2(ca.sin(X[2, self.N] - P[ref_idx + 2]),
                               ca.cos(X[2, self.N] - P[ref_idx + 2]))
        obj += self.Q_theta * 3.0 * theta_err_N**2
        
        # === 构建 NLP ===
        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        g = ca.vertcat(*g)
        
        nlp = {'f': obj, 'x': opt_vars, 'g': g, 'p': P}
        
        opts = {
            'ipopt': {
                'max_iter': 50,
                'print_level': 0,
                'acceptable_tol': 1e-4,
                'warm_start_init_point': 'yes'
            },
            'print_time': 0
        }
        
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # 保存维度信息
        self.n_state = n_state
        self.n_control = n_control
        self.n_ref_per_point = n_ref_per_point
        
        # 约束边界
        self._setup_bounds()
    
    def _setup_bounds(self):
        """设置优化变量边界"""
        self.lbx = []
        self.ubx = []
        
        # 状态无界
        for _ in range(self.N + 1):
            self.lbx += [-ca.inf, -ca.inf, -ca.inf]
            self.ubx += [ca.inf, ca.inf, ca.inf]
        
        # 控制有界
        for _ in range(self.N):
            self.lbx += [self.v_min, -self.w_max]
            self.ubx += [self.v_max, self.w_max]
        
        # 等式约束 (动力学)
        n_constraints = self.n_state * (self.N + 1)
        self.lbg = [0] * n_constraints
        self.ubg = [0] * n_constraints
    
    def _init_warm_start(self):
        """初始化 warm start 变量"""
        self.u0 = np.zeros((self.N, self.n_control))
        self.x0 = np.zeros((self.N + 1, self.n_state))
        self.last_control = np.array([0.0, 0.0])
    
    def solve(self, current_state, reference_window):
        """
        求解 MPC 优化问题
        
        Args:
            current_state: 当前状态 [x, y, theta]
            reference_window: 参考点列表 [{x, y, theta, v, ...}, ...]
            
        Returns:
            control: 最优控制 [v, omega]
            predicted_traj: 预测状态轨迹 [[x, y, theta], ...]
        """
        if reference_window is None or len(reference_window) < self.N + 1:
            _log_warn("[MPCTracker] Insufficient reference points!")
            return self.last_control.copy(), None
        
        # 构建参数向量
        p = list(current_state)
        
        for i in range(self.N + 1):
            ref = reference_window[min(i, len(reference_window)-1)]
            p.extend([ref['x'], ref['y'], ref['theta'], ref['v']])
        
        p.extend(self.last_control.tolist())
        p = np.array(p)
        
        # 初始猜测 (warm start)
        x0_guess = np.concatenate([self.x0.flatten(), self.u0.flatten()])
        
        try:
            sol = self.solver(
                x0=x0_guess,
                lbx=self.lbx, ubx=self.ubx,
                lbg=self.lbg, ubg=self.ubg,
                p=p
            )
            
            # 提取解
            opt = sol['x'].full().flatten()
            n_state_vars = self.n_state * (self.N + 1)
            x_opt = opt[:n_state_vars].reshape(self.N + 1, self.n_state)
            u_opt = opt[n_state_vars:].reshape(self.N, self.n_control)
            
            # 更新 warm start
            self.x0[:-1, :] = x_opt[1:, :]
            self.x0[-1, :] = x_opt[-1, :]
            self.u0[:-1, :] = u_opt[1:, :]
            self.u0[-1, :] = u_opt[-1, :]
            self.last_control = u_opt[0].copy()
            
            return u_opt[0], x_opt
            
        except Exception as e:
            _log_warn(f"[MPCTracker] Solve failed: {e}")
            return self.last_control.copy(), None
    
    def reset(self):
        """重置控制器状态"""
        self._init_warm_start()


# ==================== 测试代码 ====================

def _test():
    """单元测试"""
    print("=" * 50)
    print("MPCTracker 测试")
    print("=" * 50)
    
    # 初始化控制器
    mpc = MPCTracker(dt=0.1, N=10)
    
    # 当前状态
    current_state = np.array([0.0, 0.0, 0.0])
    
    # 创建参考轨迹 (直线)
    reference = []
    for i in range(15):
        reference.append({
            'x': i * 0.1,
            'y': 0.0,
            'theta': 0.0,
            'v': 0.15,
            'omega': 0.0,
            't': i * 0.1
        })
    
    # 求解
    control, pred_traj = mpc.solve(current_state, reference)
    
    if control is not None:
        print(f"✓ MPC 求解成功")
        print(f"  控制: v={control[0]:.3f} m/s, ω={control[1]:.3f} rad/s")
        
        if pred_traj is not None:
            print(f"  预测轨迹: {len(pred_traj)} 步")
            print(f"  预测终点: ({pred_traj[-1][0]:.2f}, {pred_traj[-1][1]:.2f})")
    else:
        print("✗ MPC 求解失败")
    
    # 测试重置
    mpc.reset()
    print("✓ 控制器已重置")
    
    # 测试连续求解
    print("\n--- 连续求解测试 ---")
    state = np.array([0.0, 0.0, 0.0])
    for step in range(5):
        ctrl, _ = mpc.solve(state, reference)
        # 简单前向模拟
        state[0] += ctrl[0] * np.cos(state[2]) * 0.1
        state[1] += ctrl[0] * np.sin(state[2]) * 0.1
        state[2] += ctrl[1] * 0.1
        print(f"  Step {step+1}: pos=({state[0]:.2f}, {state[1]:.2f}), v={ctrl[0]:.2f}")
    
    print("=" * 50)


if __name__ == '__main__':
    _test()
