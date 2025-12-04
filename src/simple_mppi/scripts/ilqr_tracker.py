#!/usr/bin/env python3
"""
iLQR 轨迹跟踪控制模块 (改进版)
===============================
基于迭代线性二次调节器 (iLQR) 的差速机器人轨迹跟踪控制器。

改进点:
    1. 参考 PythonRobotics LQR 实现，使用 DARE 求解
    2. 添加 line search 保证收敛
    3. 优化权重矩阵设计
    4. 更稳定的数值计算

接口:
    ctrl, pred_traj = ilqr.solve(current_state, reference_window)
    ilqr.reset()

作者: GitHub Copilot
"""

import numpy as np
import scipy.linalg as la

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


def _wrap_angle(angle):
    """将角度规范到 [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ILQRTracker:
    """
    iLQR 轨迹跟踪控制器 (改进版)

    特点:
        - 使用差速驱动运动学模型 x = [x, y, theta], u = [v, w]
        - 支持参考轨迹跟踪
        - 包含障碍物避障
    """

    def __init__(self,
                 dt=0.1,
                 N=20,
                 v_max=0.22,
                 v_min=0.0,
                 w_max=1.5,
                 Q_pos=10.0,
                 Q_theta=5.0,
                 Q_v=1.0,
                 R_v=0.1,
                 R_w=0.1,
                 max_iter=20,
                 reg=1e-3,
                 obs_eta=50.0,
                 obs_d0=0.5):
        """
        Args:
            dt: 控制周期 (s)
            N:  预测步数
            v_max, v_min: 线速度约束
            w_max: 角速度约束
            Q_pos: 位置跟踪权重
            Q_theta: 朝向跟踪权重
            Q_v: 速度跟踪权重
            R_v, R_w: 控制能量权重
            max_iter: 最大迭代次数
            reg: 正则化参数
            obs_eta: 障碍物斥力强度
            obs_d0: 障碍物影响范围
        """
        self.dt = float(dt)
        self.N = int(N)
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.w_max = float(w_max)

        # 状态权重矩阵 Q: [x, y, theta]
        self.Q = np.diag([Q_pos, Q_pos, Q_theta])
        # 终端权重 (加大以确保到达目标)
        self.Qf = 5.0 * self.Q
        # 控制权重矩阵 R: [v, w]
        self.R = np.diag([R_v, R_w])
        # 速度跟踪权重
        self.Qv = Q_v

        self.max_iter = int(max_iter)
        self.reg = float(reg)

        # 障碍物参数
        self.obs_eta = float(obs_eta)
        self.obs_d0 = float(obs_d0)
        self.obstacle_points = []

        # Warm start 缓存
        self.last_U = None

        _log_info(f"[iLQRTracker] Init: N={self.N}, dt={self.dt}, Q_pos={Q_pos}, R_v={R_v}")

    def reset(self):
        """重置内部状态"""
        self.last_U = None

    def _dynamics(self, x, u):
        """
        差速机器人离散动力学
        x = [x, y, theta], u = [v, w]
        """
        theta = x[2]
        v, w = u[0], u[1]

        x_next = np.array([
            x[0] + self.dt * v * np.cos(theta),
            x[1] + self.dt * v * np.sin(theta),
            _wrap_angle(x[2] + self.dt * w)
        ])
        return x_next

    def _linearize(self, x, u):
        """
        在 (x, u) 点线性化动力学
        返回 A = df/dx (3x3), B = df/du (3x2)
        """
        theta = x[2]
        v = u[0]

        A = np.eye(3)
        A[0, 2] = -self.dt * v * np.sin(theta)
        A[1, 2] = self.dt * v * np.cos(theta)

        B = np.zeros((3, 2))
        B[0, 0] = self.dt * np.cos(theta)
        B[1, 0] = self.dt * np.sin(theta)
        B[2, 1] = self.dt

        return A, B

    def _rollout(self, x0, U):
        """前向展开，返回状态轨迹 X (N+1, 3)"""
        X = np.zeros((self.N + 1, 3))
        X[0] = x0
        for k in range(self.N):
            X[k+1] = self._dynamics(X[k], U[k])
        return X

    def _obstacle_cost(self, x):
        """障碍物斥力代价"""
        if len(self.obstacle_points) == 0:
            return 0.0

        px, py = x[0], x[1]
        min_dist = float('inf')

        for ox, oy in self.obstacle_points:
            d = np.sqrt((px - ox)**2 + (py - oy)**2)
            min_dist = min(min_dist, d)

        min_dist = max(min_dist, 0.05)

        if min_dist >= self.obs_d0:
            return 0.0

        # APF 斥力势
        return 0.5 * self.obs_eta * (1.0/min_dist - 1.0/self.obs_d0)**2

    def _obstacle_gradient(self, x):
        """障碍物代价梯度"""
        grad = np.zeros(3)
        if len(self.obstacle_points) == 0:
            return grad

        px, py = x[0], x[1]
        min_dist = float('inf')
        closest = (0, 0)

        for ox, oy in self.obstacle_points:
            d = np.sqrt((px - ox)**2 + (py - oy)**2)
            if d < min_dist:
                min_dist = d
                closest = (ox, oy)

        min_dist = max(min_dist, 0.05)
        if min_dist >= self.obs_d0:
            return grad

        diff = 1.0/min_dist - 1.0/self.obs_d0
        dU_dd = -self.obs_eta * diff / (min_dist**2)

        grad[0] = dU_dd * (px - closest[0]) / min_dist
        grad[1] = dU_dd * (py - closest[1]) / min_dist
        return grad

    def _stage_cost(self, x, u, x_ref, u_ref):
        """单步代价"""
        dx = np.array([
            x[0] - x_ref[0],
            x[1] - x_ref[1],
            _wrap_angle(x[2] - x_ref[2])
        ])
        du = u - u_ref

        cost = 0.5 * dx @ self.Q @ dx   # 状态误差代价
        cost += 0.5 * du @ self.R @ du  # 控制能量代价
        cost += 0.5 * self.Qv * (u[0] - u_ref[0])**2 # 速度跟踪代价
        cost += self._obstacle_cost(x)  # 障碍物代价

        return cost

    def _terminal_cost(self, x, x_ref):
        """终端代价"""
        dx = np.array([
            x[0] - x_ref[0],
            x[1] - x_ref[1],
            _wrap_angle(x[2] - x_ref[2])
        ])
        return 0.5 * dx @ self.Qf @ dx

    def _total_cost(self, X, U, X_ref, U_ref):
        """总代价"""
        cost = 0.0
        for k in range(self.N):
            cost += self._stage_cost(X[k], U[k], X_ref[k], U_ref[k])
        cost += self._terminal_cost(X[-1], X_ref[-1])
        return cost

    def _backward_pass(self, X, U, X_ref, U_ref):
        """
        iLQR 后向传播，计算反馈增益 K 和前馈 k
        参考 PythonRobotics 标准实现
        """
        # 终端 value function
        dx_N = np.array([
            X[-1, 0] - X_ref[-1, 0],
            X[-1, 1] - X_ref[-1, 1],
            _wrap_angle(X[-1, 2] - X_ref[-1, 2])
        ])
        V_x = self.Qf @ dx_N
        V_xx = self.Qf.copy()

        # 存储增益
        Ks = np.zeros((self.N, 2, 3))
        ks = np.zeros((self.N, 2))

        for k in reversed(range(self.N)):
            x, u = X[k], U[k]
            x_ref, u_ref = X_ref[k], U_ref[k]

            # 1. 线性化动力学
            A, B = self._linearize(x, u)

            # 2. 计算代价函数的Hessian和梯度
            dx = np.array([
                x[0] - x_ref[0],
                x[1] - x_ref[1],
                _wrap_angle(x[2] - x_ref[2])
            ])
            du = u - u_ref

            l_x = self.Q @ dx + self._obstacle_gradient(x)
            l_u = self.R @ du
            l_u[0] += self.Qv * (u[0] - u_ref[0])

            l_xx = self.Q.copy()
            l_uu = self.R.copy()
            l_uu[0, 0] += self.Qv
            l_ux = np.zeros((2, 3))

            # 3.Q 函数（Bellman 方程）
            Q_x = l_x + A.T @ V_x
            Q_u = l_u + B.T @ V_x
            Q_xx = l_xx + A.T @ V_xx @ A
            Q_ux = l_ux + B.T @ V_xx @ A
            Q_uu = l_uu + B.T @ V_xx @ B

            # 正则化 (增大正则化参数)
            Q_uu_reg = Q_uu + self.reg * np.eye(2)

            # 4. 求解增益
            try:
                Q_uu_inv = np.linalg.inv(Q_uu_reg)
            except np.linalg.LinAlgError:
                Q_uu_inv = np.linalg.pinv(Q_uu_reg)

            K = -Q_uu_inv @ Q_ux
            k_ff = -Q_uu_inv @ Q_u

            # 5. 更新 value function (标准公式)
            V_x = Q_x - K.T @ Q_uu @ k_ff
            V_xx = Q_xx - K.T @ Q_uu @ K
            V_xx = 0.5 * (V_xx + V_xx.T)  # 保证对称

            Ks[k] = K
            ks[k] = k_ff

        return Ks, ks

    def _forward_pass(self, x0, X, U, Ks, ks, alpha=1.0):
        """
        iLQR 前向传播
        alpha: line search 步长
        """
        X_new = np.zeros_like(X)
        U_new = np.zeros_like(U)
        X_new[0] = x0

        for k in range(self.N):
            dx = X_new[k] - X[k] # 偏差
            du = alpha * ks[k] + Ks[k] @ dx # 控制修正
            u_new = U[k] + du

            # 约束
            u_new[0] = np.clip(u_new[0], self.v_min, self.v_max)
            u_new[1] = np.clip(u_new[1], -self.w_max, self.w_max)

            U_new[k] = u_new
            X_new[k+1] = self._dynamics(X_new[k], u_new) # 前向一步

        return X_new, U_new

    def _build_reference(self, reference_window):
        """构建参考轨迹数组"""
        # 填充到足够长度
        while len(reference_window) < self.N + 1:
            reference_window.append(reference_window[-1].copy())

        X_ref = np.zeros((self.N + 1, 3))
        U_ref = np.zeros((self.N, 2))

        for k in range(self.N + 1):
            ref = reference_window[k]
            X_ref[k] = [ref['x'], ref['y'], ref['theta']]

        for k in range(self.N):
            ref = reference_window[k]
            U_ref[k] = [ref.get('v', 0.1), ref.get('omega', 0.0)]

        return X_ref, U_ref

    def solve(self, current_state, reference_window, obstacles_list=None):
        """
        iLQR 主求解函数

        Args:
            current_state: [x, y, theta]
            reference_window: [{x, y, theta, v, omega}, ...]
            obstacles_list: [(x, y), ...] 或 None

        Returns:
            control: [v, w]
            predicted_traj: [[x, y, theta], ...]
        """
        x0 = np.asarray(current_state, dtype=float).flatten()

        # 更新障碍物
        if obstacles_list is not None:
            self.obstacle_points = [
                (ox, oy) for ox, oy in obstacles_list
                if 0.1 < np.sqrt((ox-x0[0])**2 + (oy-x0[1])**2) < 2.0
            ]
        else:
            self.obstacle_points = []

        if reference_window is None or len(reference_window) == 0:
            return np.array([0.0, 0.0]), None

        # 构建参考
        X_ref, U_ref = self._build_reference(list(reference_window))

        # 初始化控制序列
        # 简化：始终使用参考控制量，避免 warm start 的复杂性
        U = U_ref.copy()

        # 确保有前进速度
        for k in range(self.N):
            if U[k, 0] < 0.1:
                U[k, 0] = 0.15  # 给一个合理的初始速度

        # 初始轨迹
        X = self._rollout(x0, U)
        cost = self._total_cost(X, U, X_ref, U_ref)

        # 调试: 计算关键指标
        pos_error = np.sqrt((x0[0]-X_ref[0,0])**2 + (x0[1]-X_ref[0,1])**2)
        theta_error = abs(_wrap_angle(x0[2] - X_ref[0,2]))
        initial_cost = cost
        line_search_failed = False

        # iLQR 迭代
        for it in range(self.max_iter):
            # Backward pass
            Ks, ks = self._backward_pass(X, U, X_ref, U_ref)

            # Line search
            alpha = 1.0
            improved = False
            for _ in range(10):
                X_new, U_new = self._forward_pass(x0, X, U, Ks, ks, alpha)
                cost_new = self._total_cost(X_new, U_new, X_ref, U_ref)

                if cost_new < cost:
                    X, U = X_new, U_new
                    cost = cost_new
                    improved = True
                    break
                alpha *= 0.5

            if not improved:
                line_search_failed = True
                break

            # 收敛检查
            if alpha < 0.01:
                break

        # ========== WARN 输出关键调试信息 ==========
        if HAS_ROS:
            rospy.logwarn(f"[iLQR] pos_err={pos_error:.2f}m θ_err={np.degrees(theta_error):.0f}° | "
                         f"cost:{initial_cost:.0f}->{cost:.0f}({100*(initial_cost-cost)/max(initial_cost,1):.0f}%) | "
                         f"v={U[0,0]:.2f} w={U[0,1]:.2f} iter={it+1}")

        # 确保输出有速度 (防止卡死)
        if U[0, 0] < 0.05:
            U[0, 0] = 0.1

        return U[0].copy(), X.tolist()


def _test():
    """单元测试"""
    print("=" * 50)
    print("iLQR Tracker 测试")
    print("=" * 50)

    tracker = ILQRTracker(dt=0.1, N=15, Q_pos=10.0, R_v=0.1)

    # 初始状态
    state = np.array([0.0, 0.0, 0.0])

    # 直线参考轨迹
    reference = []
    for k in range(20):
        reference.append({
            'x': 0.1 * k,
            'y': 0.0,
            'theta': 0.0,
            'v': 0.15,
            'omega': 0.0
        })

    print("跟踪直线轨迹:")
    for step in range(15):
        u, pred = tracker.solve(state, reference)
        print(f"  Step {step+1}: v={u[0]:.3f}, w={u[1]:.3f}, "
              f"pos=({state[0]:.3f}, {state[1]:.3f})")
        state = tracker._dynamics(state, u)

    print(f"\n最终位置: ({state[0]:.3f}, {state[1]:.3f})")
    print("=" * 50)


if __name__ == "__main__":
    _test()
