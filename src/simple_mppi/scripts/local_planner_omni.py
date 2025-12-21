#!/usr/bin/env python3
"""
局部轨迹规划模块 - 全向移动版本
================================
将几何路径转换为时间参数化轨迹

与差速版本的区别:
    - 输出包含 vx, vy 两个速度分量
    - 全向平台可以独立控制朝向

作者: GitHub Copilot
"""

import numpy as np
from scipy import interpolate

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


class TrajectoryGeneratorOmni:
    """
    轨迹生成器 - 全向移动版本

    输出格式:
        {'x', 'y', 'theta', 'vx', 'vy', 'omega', 't'}
    """

    def __init__(self, v_max=0.5, v_min=0.05, w_max=1.0, a_max=0.8, alpha_max=1.0):
        """
        Args:
            v_max: 最大线速度 (m/s)
            v_min: 最小线速度 (m/s)
            w_max: 最大角速度 (rad/s)
            a_max: 最大线加速度 (m/s^2)
            alpha_max: 最大角加速度 (rad/s^2)
        """
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        self.a_max = a_max
        self.alpha_max = alpha_max

        self._spline_headings = None
        self._spline_curvatures = None

        _log_info(f"[TrajectoryGeneratorOmni] Init: v_max={v_max}, a_max={a_max}")

    def generate(self, geometric_path, current_state, dt=0.1):
        """
        生成时间参数化轨迹

        Args:
            geometric_path: 几何路径 [(x, y), ...]
            current_state: 当前状态 [x, y, theta]
            dt: 时间步长

        Returns:
            轨迹列表 [{'x', 'y', 'theta', 'vx', 'vy', 'omega', 't'}, ...]
        """
        if geometric_path is None or len(geometric_path) < 2:
            return None

        self._spline_headings = None
        self._spline_curvatures = None

        # Step 1: B样条平滑
        dense_path = self._smooth_path_spline(geometric_path, spacing=0.05)

        # Step 2: 曲率
        if self._spline_curvatures is not None and len(self._spline_curvatures) == len(dense_path):
            curvatures = list(self._spline_curvatures)
        else:
            curvatures = self._compute_curvatures(dense_path)

        # Step 3: 速度规划
        speeds = self._plan_speeds(dense_path, curvatures)

        # Step 4: 构建轨迹
        trajectory = self._build_trajectory(dense_path, speeds, current_state, dt)

        # Step 5: 停止段
        trajectory = self._add_stop_segment(trajectory, dt)

        total_time = trajectory[-1]['t'] if trajectory else 0
        _log_info(f"[TrajectoryGeneratorOmni] Trajectory: {len(trajectory)} pts, {total_time:.1f}s")

        return trajectory

    def _smooth_path_spline(self, path, spacing=0.05, use_bspline=True):
        """B样条平滑"""
        if len(path) < 4:
            self._spline_headings = None
            self._spline_curvatures = None
            return self._interpolate_path(path, spacing)

        x = np.array([p[0] for p in path])
        y = np.array([p[1] for p in path])

        dx, dy = np.diff(x), np.diff(y)
        distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
        distances = np.concatenate([[0.0], distances])
        total_length = distances[-1]

        if total_length < 0.1:
            self._spline_headings = None
            self._spline_curvatures = None
            return list(path)

        distances_norm = distances / total_length

        try:
            smoothing = len(path) * 0.01 if use_bspline else 0.0
            degree = min(3, len(path) - 1)

            spl_x = interpolate.UnivariateSpline(distances_norm, x, k=degree, s=smoothing)
            spl_y = interpolate.UnivariateSpline(distances_norm, y, k=degree, s=smoothing)

            n_points = max(int(total_length / spacing), 2)
            sampled = np.linspace(0.0, 1.0, n_points)

            x_new = spl_x(sampled)
            y_new = spl_y(sampled)

            # 解析求导
            dx_ds = spl_x.derivative(1)(sampled)
            dy_ds = spl_y.derivative(1)(sampled)
            headings = np.arctan2(dy_ds, dx_ds)

            ddx_ds = spl_x.derivative(2)(sampled)
            ddy_ds = spl_y.derivative(2)(sampled)
            curvatures = (dy_ds * ddx_ds - dx_ds * ddy_ds) / \
                         np.power(dx_ds**2 + dy_ds**2 + 1e-9, 1.5)

            self._spline_headings = headings
            self._spline_curvatures = np.abs(curvatures)

            return [(x_new[i], y_new[i]) for i in range(len(x_new))]

        except Exception as e:
            _log_info(f"[TrajectoryGeneratorOmni] Spline failed: {e}")
            self._spline_headings = None
            self._spline_curvatures = None
            return self._interpolate_path(path, spacing)

    def _interpolate_path(self, path, spacing=0.05):
        """线性插值"""
        if len(path) < 2:
            return list(path)

        dense = [path[0]]
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            dist = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            n_points = max(2, int(dist / spacing))

            for j in range(1, n_points + 1):
                alpha = j / n_points
                x = p1[0] + alpha * (p2[0] - p1[0])
                y = p1[1] + alpha * (p2[1] - p1[1])
                dense.append((x, y))

        return dense

    def _compute_curvatures(self, path):
        """计算曲率"""
        n = len(path)
        curvatures = [0.0] * n

        for i in range(1, n - 1):
            p0, p1, p2 = path[i-1], path[i], path[i+1]

            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])

            angle1 = np.arctan2(v1[1], v1[0])
            angle2 = np.arctan2(v2[1], v2[0])
            dangle = abs(self._normalize_angle(angle2 - angle1))

            ds = (np.sqrt(v1[0]**2 + v1[1]**2) + np.sqrt(v2[0]**2 + v2[1]**2)) / 2
            curvatures[i] = dangle / ds if ds > 0.001 else 0.0

        return curvatures

    def _plan_speeds(self, path, curvatures):
        """速度规划"""
        n = len(path)
        if n < 2:
            return [0.0] * n

        # 基于曲率
        v_curvature = np.zeros(n)
        curvature_gain = 2.0  # 全向对曲率不太敏感

        for i in range(n):
            v_curvature[i] = self.v_max / np.sqrt(1 + curvature_gain * abs(curvatures[i]))
            v_curvature[i] = np.clip(v_curvature[i], self.v_min, self.v_max)

        v_curvature[-1] = 0.0
        v_curvature[-2] = 0.0 if n > 1 else 0.0

        # 后向传播
        speeds = v_curvature.copy()
        for i in range(n - 2, -1, -1):
            ds = np.sqrt((path[i+1][0] - path[i][0])**2 +
                         (path[i+1][1] - path[i][1])**2)
            v_decel = np.sqrt(speeds[i+1]**2 + 2 * self.a_max * ds)
            speeds[i] = min(speeds[i], v_decel)

        # 前向传播
        for i in range(n - 1):
            ds = np.sqrt((path[i+1][0] - path[i][0])**2 +
                         (path[i+1][1] - path[i][1])**2)
            v_accel = np.sqrt(speeds[i]**2 + 2 * self.a_max * ds)
            speeds[i+1] = min(speeds[i+1], v_accel)

        speeds[0] = min(speeds[0], self.v_min)

        return list(speeds)

    def _build_trajectory(self, path, speeds, current_state, dt):
        """构建轨迹"""
        trajectory = []
        t = 0.0

        use_analytical = (self._spline_headings is not None and
                          len(self._spline_headings) == len(path))

        for i in range(len(path)):
            x, y = path[i]
            v = speeds[i]

            # 朝向
            if use_analytical:
                theta = self._spline_headings[i]
            elif i < len(path) - 1:
                dx = path[i+1][0] - x
                dy = path[i+1][1] - y
                theta = np.arctan2(dy, dx)
            else:
                theta = trajectory[-1]['theta'] if trajectory else current_state[2]

            # 全向: 主要使用前向速度 vx, vy=0
            # 如果需要横向运动，可在此处计算 vy
            vx = v
            vy = 0.0

            # 角速度
            if i > 0:
                dtheta = self._normalize_angle(theta - trajectory[-1]['theta'])
                dt_actual = t - trajectory[-1]['t']
                if dt_actual > 0.001:
                    omega = np.clip(dtheta / dt_actual, -self.w_max, self.w_max)
                else:
                    omega = np.clip(dtheta / dt, -self.w_max, self.w_max)
            else:
                omega = 0.0

            trajectory.append({
                'x': x, 'y': y, 'theta': theta,
                'vx': vx, 'vy': vy,
                'v': v,  # 兼容
                'omega': omega, 't': t
            })

            # 时间步进
            if i < len(path) - 1 and v > 0.01:
                dist = np.sqrt((path[i+1][0] - x)**2 + (path[i+1][1] - y)**2)
                t += dist / max(v, 0.01)
            else:
                t += dt

        return trajectory

    def _add_stop_segment(self, trajectory, dt, n_stop=10):
        """添加停止段"""
        if not trajectory:
            return trajectory

        last = trajectory[-1]
        t = last['t']

        for _ in range(n_stop):
            t += dt
            trajectory.append({
                'x': last['x'], 'y': last['y'], 'theta': last['theta'],
                'vx': 0.0, 'vy': 0.0, 'v': 0.0, 'omega': 0.0, 't': t
            })

        return trajectory

    def _normalize_angle(self, angle):
        """角度归一化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


# ==================== 测试 ====================

def _test():
    print("=" * 50)
    print("TrajectoryGeneratorOmni 测试")
    print("=" * 50)

    path = [(0, 0), (1, 0), (2, 0.5), (3, 1), (3, 2)]
    current_state = np.array([0, 0, 0])

    generator = TrajectoryGeneratorOmni(v_max=0.5, v_min=0.05, w_max=1.0)
    traj = generator.generate(path, current_state, dt=0.1)

    if traj:
        print(f"轨迹生成成功: {len(traj)} 点")
        print(f"  总时长: {traj[-1]['t']:.2f}s")
        print(f"  起点: ({traj[0]['x']:.2f}, {traj[0]['y']:.2f}), vx={traj[0]['vx']:.2f}")
        print(f"  终点: ({traj[-1]['x']:.2f}, {traj[-1]['y']:.2f})")
    else:
        print("轨迹生成失败")

    print("=" * 50)


if __name__ == '__main__':
    _test()
