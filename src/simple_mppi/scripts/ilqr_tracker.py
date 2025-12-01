#!/usr/bin/env python3
"""
iLQR 轨迹跟踪控制模块 (Layer 3)
===============================
基于迭代线性二次调节器 (iLQR) 的差速机器人轨迹跟踪控制器。

特点:
    - 使用差速驱动运动学模型 x = [x, y, theta], u = [v, w]
    - 以 Layer2 生成的参考轨迹窗口为 nominal 轨迹
    - 每次调用在当前状态附近做若干次 iLQR 迭代，输出第一步控制
    - 接口尽量与 MPCTracker / MPPITracker 保持一致:

        ctrl, pred_traj = ilqr.solve(current_state, reference_window)
        ilqr.reset()
"""

import numpy as np

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
    """将角度规范到 [-pi, pi]。"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ILQRTracker:
    """
    iLQR 轨迹跟踪控制器
    """

    def __init__(self,
                 dt=0.1,
                 N=20,
                 v_max=0.22,
                 v_min=-0.05,
                 w_max=1.8,
                 Q_pos=10.0,
                 Q_theta=5.0,
                 Q_v=2.0,
                 R_v=0.05,
                 R_w=0.05,
                 max_iter=5,
                 reg=1e-6):
        """
        Args:
            dt: 控制周期 (s)
            N:  预测步数
            v_max, v_min: 线速度约束
            w_max: 角速度约束
            Q_pos: 位置跟踪权重 (作用在 x, y 上)
            Q_theta: 朝向跟踪权重
            Q_v: 速度跟踪权重 (作用在 v 上)
            R_v, R_w: 控制能量权重
            max_iter: 每次 solve 的最大 iLQR 迭代次数
            reg: Q_uu 正定化的正则项
        """
        self.dt = float(dt)
        self.N = int(N)
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.w_max = float(w_max)

        # 状态误差: [x_err, y_err, theta_err]
        self.Q = np.diag([Q_pos, Q_pos, Q_theta])
        # 终端权重稍大一点
        self.Qf = 3.0 * self.Q
        # 控制误差: [v_err, w_err]
        self.R = np.diag([R_v, R_w])
        # 速度参考误差的附加权重
        self.Qv = Q_v

        self.max_iter = int(max_iter)
        self.reg = float(reg)

        # 上一次求解得到的控制 / 状态轨迹, 用于 warm start
        self.last_U = np.zeros((self.N, 2), dtype=float)
        self.last_X = None
        self.last_control = np.array([0.0, 0.0], dtype=float)

        _log_info(f"[iLQRTracker] Initialized: N={self.N}, dt={self.dt}")

    # ====== 机器人动力学与线性化 ======

    def _f(self, x, u):
        """
        差速机器人离散动力学:
            x_{k+1} = f(x_k, u_k)
        x = [x, y, theta]
        u = [v, w]
        """
        nx, ny, theta = x
        v, w = u

        nx_next = nx + self.dt * v * np.cos(theta)
        ny_next = ny + self.dt * v * np.sin(theta)
        theta_next = theta + self.dt * w
        theta_next = _wrap_angle(theta_next)

        return np.array([nx_next, ny_next, theta_next])

    def _linearize(self, x, u):
        """
        在 (x, u) 点对动力学 f 进行线性化,
        得到 A = df/dx, B = df/du, 维度分别为 (3,3), (3,2)。
        """
        _, _, theta = x
        v, _ = u

        dt = self.dt
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        # df/dx
        A = np.eye(3)
        A[0, 2] = -dt * v * sin_t
        A[1, 2] = dt * v * cos_t

        # df/du
        B = np.zeros((3, 2))
        B[0, 0] = dt * cos_t
        B[1, 0] = dt * sin_t
        B[2, 1] = dt

        return A, B

    # ====== 公共接口 ======

    def reset(self):
        """重置内部状态 (例如切换目标时调用)。"""
        self.last_U = np.zeros((self.N, 2), dtype=float)
        self.last_X = None
        self.last_control = np.array([0.0, 0.0], dtype=float)

    def _build_reference_arrays(self, reference_window):
        """
        将参考窗口 (list of dict) 转为 numpy 数组形式:

        X_ref: (N+1, 3) -> [x_ref, y_ref, theta_ref]
        U_ref: (N,   2) -> [v_ref, w_ref]
        """
        # 如果参考长度不足, 简单重复最后一个
        if len(reference_window) < self.N + 1:
            last = reference_window[-1]
            while len(reference_window) < self.N + 1:
                reference_window.append(last)

        # 状态参考
        X_ref = np.zeros((self.N + 1, 3), dtype=float)
        for k in range(self.N + 1):
            ref = reference_window[k]
            X_ref[k, 0] = ref['x']
            X_ref[k, 1] = ref['y']
            X_ref[k, 2] = ref['theta']

        # 控制参考 (用 v_ref, omega_ref)
        U_ref = np.zeros((self.N, 2), dtype=float)
        for k in range(self.N):
            ref = reference_window[min(k, len(reference_window) - 1)]
            U_ref[k, 0] = ref.get('v', 0.0)
            U_ref[k, 1] = ref.get('omega', 0.0)

        return X_ref, U_ref

    def _rollout(self, x0, U):
        """
        根据初始状态 x0 和控制序列 U (N,2) 前向展开,
        返回状态轨迹 X (N+1,3)。
        """
        X = np.zeros((self.N + 1, 3), dtype=float)
        X[0] = x0
        for k in range(self.N):
            X[k + 1] = self._f(X[k], U[k])
        return X

    def _stage_cost(self, x, u, x_ref, u_ref):
        """
        单步 cost:
            l = 0.5 * (x - x_ref)^T Q (x - x_ref)
              + 0.5 * (u - u_ref)^T R (u - u_ref)
              + 0.5 * Qv * (v - v_ref)^2
        """
        dx = np.array([
            x[0] - x_ref[0],
            x[1] - x_ref[1],
            _wrap_angle(x[2] - x_ref[2])
        ])
        du = u - u_ref

        l_state = 0.5 * dx.T @ self.Q @ dx
        l_control = 0.5 * du.T @ self.R @ du

        # 速度误差额外权重
        v_err = u[0] - u_ref[0]
        l_v = 0.5 * self.Qv * v_err * v_err

        return l_state + l_control + l_v

    def _terminal_cost(self, x, x_ref, v_ref):
        """
        终端 cost:
            lN = 0.5 * (x - x_ref)^T Qf (x - x_ref)
               + 0.5 * Qv * (0 - v_ref)^2   # 希望终端速度接近 0 或参考值
        """
        dx = np.array([
            x[0] - x_ref[0],
            x[1] - x_ref[1],
            _wrap_angle(x[2] - x_ref[2])
        ])
        l_state = 0.5 * dx.T @ self.Qf @ dx

        # 这里简单处理: 希望终端速度接近 0
        l_v = 0.5 * self.Qv * (0.0 - v_ref) ** 2
        return l_state + l_v

    def _trajectory_cost(self, X, U, X_ref, U_ref):
        """辅助: 计算完整轨迹的总 cost, 用于调试/线搜索。"""
        cost = 0.0
        for k in range(self.N):
            cost += self._stage_cost(X[k], U[k], X_ref[k], U_ref[k])
        cost += self._terminal_cost(X[-1], X_ref[-1], U_ref[-1, 0])
        return cost

    def solve(self, current_state, reference_window):
        """
        iLQR 主入口。

        Args:
            current_state: 当前状态 [x, y, theta] (list / np.ndarray)
            reference_window: 参考点列表 [{x, y, theta, v, omega, t}, ...]
        Returns:
            control: 第一时刻控制 [v, w]
            predicted_traj: 预测状态轨迹 [[x, y, theta], ...] 或 None
        """
        current_state = np.asarray(current_state, dtype=float).flatten()
        if current_state.shape[0] != 3:
            raise ValueError("current_state 必须是长度为 3 的向量 [x, y, theta]")

        if reference_window is None or len(reference_window) == 0:
            # 没有参考时保持上次控制
            return self.last_control.copy(), None

        # 构建参考数组
        X_ref, U_ref = self._build_reference_arrays(list(reference_window))

        # 初始化控制序列 U:
        #   - 如果上一轮有结果, 做一个 shift; 否则用参考控制
        if self.last_X is not None:
            U = np.vstack([self.last_U[1:], self.last_U[-1:]])
        else:
            U = U_ref.copy()

        # 前向展开得到 nominal 轨迹
        X = self._rollout(current_state, U)
        cost_prev = self._trajectory_cost(X, U, X_ref, U_ref)

        # 迭代 iLQR
        for it in range(self.max_iter):
            # ---- Backward pass ----
            V_x = np.zeros(3, dtype=float)
            V_xx = np.zeros((3, 3), dtype=float)

            # 终端 cost 的梯度和 Hessian
            xN = X[-1]
            xN_ref = X_ref[-1]
            vN_ref = U_ref[-1, 0]

            dxN = np.array([
                xN[0] - xN_ref[0],
                xN[1] - xN_ref[1],
                _wrap_angle(xN[2] - xN_ref[2])
            ])
            V_x = self.Qf @ dxN
            V_xx = self.Qf.copy()

            # 为每个时间步存储反馈矩阵
            Ks = np.zeros((self.N, 2, 3), dtype=float)
            ks = np.zeros((self.N, 2), dtype=float)

            for k in reversed(range(self.N)):
                xk = X[k]
                uk = U[k]
                xr = X_ref[k]
                ur = U_ref[k]

                # 线性化动力学
                A, B = self._linearize(xk, uk)

                # Cost 的一阶/二阶导数
                dx = np.array([
                    xk[0] - xr[0],
                    xk[1] - xr[1],
                    _wrap_angle(xk[2] - xr[2])
                ])
                du = uk - ur

                l_x = self.Q @ dx
                l_u = self.R @ du
                # 速度参考项导数 (只对 v 有影响)
                v_err = uk[0] - ur[0]
                l_u[0] += self.Qv * v_err

                l_xx = self.Q
                l_uu = self.R.copy()
                l_ux = np.zeros((2, 3), dtype=float)

                # 二次近似的 Q 函数
                Q_x = l_x + A.T @ V_x
                Q_u = l_u + B.T @ V_x
                Q_xx = l_xx + A.T @ V_xx @ A
                Q_ux = l_ux + B.T @ V_xx @ A
                Q_uu = l_uu + B.T @ V_xx @ B

                # 正则化以避免 Q_uu 非正定
                Q_uu_reg = Q_uu + self.reg * np.eye(2)

                try:
                    Q_uu_inv = np.linalg.inv(Q_uu_reg)
                except np.linalg.LinAlgError:
                    Q_uu_inv = np.linalg.pinv(Q_uu_reg)

                K = -Q_uu_inv @ Q_ux       # 反馈矩阵 (2x3)
                k_ff = -Q_uu_inv @ Q_u     # 前馈向量 (2,)

                # 更新 value function
                V_x = Q_x + K.T @ Q_uu @ k_ff + K.T @ Q_u + Q_ux.T @ k_ff
                V_xx = Q_xx + K.T @ Q_uu @ K + K.T @ Q_ux + Q_ux.T @ K

                Ks[k] = K
                ks[k] = k_ff

            # ---- Forward pass ----
            X_new = np.zeros_like(X)
            U_new = np.zeros_like(U)
            X_new[0] = current_state

            for k in range(self.N):
                # 相对于 nominal 轨迹的偏差
                dx = X_new[k] - X[k]
                du = ks[k] + Ks[k] @ dx
                u_new = U[k] + du

                # 速度/角速度饱和
                u_new[0] = np.clip(u_new[0], self.v_min, self.v_max)
                u_new[1] = np.clip(u_new[1], -self.w_max, self.w_max)

                U_new[k] = u_new
                X_new[k + 1] = self._f(X_new[k], u_new)

            # 计算新轨迹的 cost
            cost_new = self._trajectory_cost(X_new, U_new, X_ref, U_ref)

            # 这里没有做复杂 line search, 简单接受新解
            X, U = X_new, U_new
            cost_prev = cost_new

        # iLQR 结束, 取第一步控制
        u0 = U[0].copy()
        self.last_control = u0
        self.last_U = U
        self.last_X = X

        # 预测轨迹给上层可视化使用
        predicted_traj = X.tolist()

        return u0, predicted_traj


def _test():
    """简单单元测试: 让机器人沿直线走一小段."""
    tracker = iLQRTracker(dt=0.1, N=15)

    # 初始状态
    state = np.array([0.0, 0.0, 0.0])

    # 构造一段简单的直线参考 (向 x 正方向 1.5m)
    reference = []
    for k in range(16):
        x_ref = 0.1 * k
        ref = {
            'x': x_ref,
            'y': 0.0,
            'theta': 0.0,
            'v': 0.2,
            'omega': 0.0,
            't': 0.1 * k
        }
        reference.append(ref)

    for step in range(10):
        u, pred = tracker.solve(state, reference)
        v, w = u
        print(f"Step {step+1}: v={v:.3f}, w={w:.3f}, state=({state[0]:.3f},{state[1]:.3f},{state[2]:.3f})")
        state = tracker._f(state, u)


if __name__ == "__main__":
    _test()
