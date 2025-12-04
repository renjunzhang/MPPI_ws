#!/usr/bin/env python3
"""
局部轨迹规划模块 (Layer 2) - 改进版
====================================
将几何路径转换为时间参数化轨迹

算法改进:
    1. 三次样条插值 (替代线性插值) - 更平滑的路径
    2. 基于优化的速度规划 - 考虑加速度约束
    3. 梯形速度曲线 - 平滑的加减速

功能:
    - 路径平滑 (三次样条)
    - 曲率计算
    - 基于曲率的速度规划 (弯道减速)
    - 梯形加减速处理
    - 时间参数化轨迹生成

输入: 几何路径 [(x, y), ...], 当前状态 [x, y, theta]
输出: 时间参数化轨迹 [{x, y, theta, v, omega, t}, ...]

作者: GitHub Copilot
"""

import numpy as np  # 引入 NumPy 用于数值计算
from scipy import interpolate   # 引入 SciPy 的插值模块，用于三次样条插值

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


class TrajectoryGenerator:  # 这个class是局部轨迹规划器的核心，就代表着局部规划器
    """
    轨迹生成器
    
    将离散的几何路径点转换为连续的时间参数化轨迹，
    包含位置、朝向、速度、角速度和时间信息。
    
    特点:
        - 边走边转，不停顿
        - 曲率自适应速度规划
        - 平滑的加减速
    
    Attributes:
        v_max (float): 最大线速度 (m/s)
        v_min (float): 最小线速度 (m/s)
        w_max (float): 最大角速度 (rad/s)
    """
    
    def __init__(self, v_max=0.20, v_min=0.05, w_max=1.5, a_max=0.5, alpha_max=1.0):
        """
        初始化轨迹生成器
        
        Args:
            v_max: 最大线速度 (m/s)
            v_min: 最小线速度 (m/s)
            w_max: 最大角速度 (rad/s)
            a_max: 最大线加速度 (m/s^2) - 新增
            alpha_max: 最大角加速度 (rad/s^2) - 新增
        """
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        self.a_max = a_max
        self.alpha_max = alpha_max
        
        # 解析计算结果缓存 (来自B样条求导)
        self._spline_headings = None    # 解析航向角
        self._spline_curvatures = None  # 解析曲率
        
        _log_info(f"[TrajectoryGenerator] Initialized (v_max={v_max}, a_max={a_max})")
    
    def generate(self, geometric_path, current_state, dt=0.1):
        """
        生成时间参数化轨迹
        
        Args:
            geometric_path: 几何路径 [(x, y), ...]
            current_state: 当前状态 [x, y, theta]
            dt: 时间步长 (s)
            
        Returns:
            轨迹列表 [{x, y, theta, v, omega, t}, ...] 或 None
        """
        if geometric_path is None or len(geometric_path) < 2:
            return None
        
        # 初始化解析结果
        self._spline_headings = None
        self._spline_curvatures = None
        
        # Step 1: 路径平滑 (B样条 + 解析求导)
        dense_path = self._smooth_path_spline(geometric_path, spacing=0.05)
        
        # Step 2: 获取曲率 (优先使用解析结果，否则数值计算)
        if self._spline_curvatures is not None and len(self._spline_curvatures) == len(dense_path):
            curvatures = list(self._spline_curvatures)
            _log_info("[TrajectoryGenerator] Using analytical curvature from spline")
        else:
            curvatures = self._compute_curvatures(dense_path)
        
        # Step 3: 速度规划 (考虑曲率和加速度约束)
        speeds = self._plan_speeds_optimized(dense_path, curvatures)
        
        # Step 4: 生成轨迹 (使用解析航向或数值计算)
        trajectory = self._build_trajectory_improved(dense_path, speeds, current_state, dt)
        
        # Step 5: 添加停止段
        trajectory = self._add_stop_segment(trajectory, dt)
        
        total_time = trajectory[-1]['t'] if trajectory else 0
        _log_info(f"[TrajectoryGenerator] Trajectory: {len(trajectory)} pts, {total_time:.1f}s")
        
        return trajectory
    
    def _smooth_path_spline(self, path, spacing=0.05, use_bspline=True):
        """
        样条平滑路径 (改进版 - 参考 PythonRobotics)
        
        使用 UnivariateSpline 进行B样条平滑，支持解析求导计算航向和曲率。
        
        Args:
            path: 原始路径 [(x, y), ...]
            spacing: 点间距 (m)
            use_bspline: True=B样条逼近(更平滑), False=精确插值
        
        Returns:
            dense_path: 密集路径点列表
            
        同时更新:
            self._spline_headings: 解析计算的航向角
            self._spline_curvatures: 解析计算的曲率
        """
        if len(path) < 4:
            # 点太少，退化为线性插值
            self._spline_headings = None
            self._spline_curvatures = None
            return self._interpolate_path(path, spacing)
        
        # 提取 x, y
        x = np.array([p[0] for p in path])
        y = np.array([p[1] for p in path])
        
        # 计算归一化弧长参数 (参考 PythonRobotics)
        dx, dy = np.diff(x), np.diff(y) # 计算差分，得到每段的增量的数组
        distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)]) # distances[i] 表示从第 1 个点走到第 i+1 个点所经过的总路程
        distances = np.concatenate([[0.0], distances])  # 对齐坐标数组 x, y 的长度
        total_length = distances[-1]
        
        if total_length < 0.1:
            self._spline_headings = None
            self._spline_curvatures = None
            return list(path)
        
        # 归一化到 [0, 1]
        distances_norm = distances / total_length
        
        try:
            # 平滑参数: use_bspline=True 时平滑, False 时精确插值
            smoothing = len(path) * 0.01 if use_bspline else 0.0
            degree = min(3, len(path) - 1)
            
            # 使用 UnivariateSpline (与 PythonRobotics 相同)
            spl_x = interpolate.UnivariateSpline(distances_norm, x, k=degree, s=smoothing)
            spl_y = interpolate.UnivariateSpline(distances_norm, y, k=degree, s=smoothing)
            
            # 均匀采样
            n_points = max(int(total_length / spacing), 2)
            sampled = np.linspace(0.0, 1.0, n_points)
            
            # 计算位置
            x_new = spl_x(sampled)
            y_new = spl_y(sampled)
            
            # 解析求导计算航向角 (比差分更准确)
            dx_ds = spl_x.derivative(1)(sampled)
            dy_ds = spl_y.derivative(1)(sampled)
            headings = np.arctan2(dy_ds, dx_ds)
            
            # 解析求导计算曲率 (比三点法更准确)
            ddx_ds = spl_x.derivative(2)(sampled)
            ddy_ds = spl_y.derivative(2)(sampled)
            # 曲率公式: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
            curvatures = (dy_ds * ddx_ds - dx_ds * ddy_ds) / \
                         np.power(dx_ds**2 + dy_ds**2 + 1e-9, 1.5)
            
            # 保存解析结果供后续使用
            self._spline_headings = headings
            self._spline_curvatures = np.abs(curvatures)  # 取绝对值
            
            return [(x_new[i], y_new[i]) for i in range(len(x_new))]
            
        except Exception as e:
            # 样条插值失败，退化为线性插值
            _log_info(f"[TrajectoryGenerator] Spline failed！！！: {e}, using linear")
            self._spline_headings = None
            self._spline_curvatures = None
            return self._interpolate_path(path, spacing)
    
    def _interpolate_path(self, path, spacing=0.05):        # spacing点间距是插入的点的间距
        """
        路径插值，生成密集点
        
        Args:
            path: 原始路径
            spacing: 点间距 (m)
        """
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
        """
        计算路径曲率
        
        使用三点法计算每个点的曲率
        """
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
        
        # 平滑曲率
        smoothed = curvatures.copy()
        for i in range(2, n - 2):
            smoothed[i] = np.mean(curvatures[i-2:i+3])
        
        return smoothed
    
    def _plan_speeds_optimized(self, path, curvatures):
        """
        优化的速度规划 (改进版)
        
        改进点:
            1. 曲率-速度约束: v <= v_max / sqrt(1 + k*|curvature|)
            2. 前向-后向传播: 满足加速度约束
            3. 梯形速度曲线: 平滑的加减速
        
        Args:
            path: 路径点
            curvatures: 曲率
        Returns:
            speeds: 速度列表
        """
        n = len(path)
        if n < 2:
            return [0.0] * n
        
        # Step 1: 基于曲率的速度上限
        v_curvature = np.zeros(n)
        curvature_gain = 3.0  # 曲率敏感度
        for i in range(n):
            # v_max / sqrt(1 + k*|curvature|) 比原来的除法更平滑
            v_curvature[i] = self.v_max / np.sqrt(1 + curvature_gain * abs(curvatures[i]))
            v_curvature[i] = np.clip(v_curvature[i], self.v_min, self.v_max)
        
        # Step 2: 终点速度为 0
        v_curvature[-1] = 0.0
        v_curvature[-2] = 0.0 if n > 1 else 0.0
        
        # Step 3: 后向传播 - 确保能减速停下来
        # v[i]^2 <= v[i+1]^2 + 2 * a_max * ds
        speeds = v_curvature.copy()
        for i in range(n - 2, -1, -1):
            ds = np.sqrt((path[i+1][0] - path[i][0])**2 + 
                         (path[i+1][1] - path[i][1])**2)
            v_decel = np.sqrt(speeds[i+1]**2 + 2 * self.a_max * ds)
            speeds[i] = min(speeds[i], v_decel)
        
        # Step 4: 前向传播 - 确保能加速到目标速度
        # v[i+1]^2 <= v[i]^2 + 2 * a_max * ds
        for i in range(n - 1):
            ds = np.sqrt((path[i+1][0] - path[i][0])**2 + 
                         (path[i+1][1] - path[i][1])**2)
            v_accel = np.sqrt(speeds[i]**2 + 2 * self.a_max * ds)
            speeds[i+1] = min(speeds[i+1], v_accel)
        
        # Step 5: 起点速度处理 (从静止开始)
        speeds[0] = min(speeds[0], self.v_min)
        
        return list(speeds)
    
    def _build_trajectory_improved(self, path, speeds, current_state, dt):
        """
        构建轨迹 (改进版 - 使用解析航向)
        
        如果有解析计算的航向角，使用它们代替差分近似。
        解析航向更准确、更平滑。
        """
        trajectory = []
        t = 0.0     # 全局时间累加器
        
        # 检查是否有解析航向
        use_analytical = (self._spline_headings is not None and 
                          len(self._spline_headings) == len(path))
        
        if use_analytical:
            _log_info("[TrajectoryGenerator] Using analytical heading from spline")
        
        for i in range(len(path)):
            x, y = path[i]
            v = speeds[i]
            
            # 计算朝向 (优先使用解析结果)
            if use_analytical:
                theta = self._spline_headings[i]
            elif i < len(path) - 1:
                dx = path[i+1][0] - x
                dy = path[i+1][1] - y
                theta = np.arctan2(dy, dx)
            else:
                theta = trajectory[-1]['theta'] if trajectory else current_state[2]
            
            # 计算角速度
            if i > 0:
                dtheta = self._normalize_angle(theta - trajectory[-1]['theta'])
                # 使用实际时间间隔计算角速度
                if trajectory:
                    dt_actual = t - trajectory[-1]['t']
                    if dt_actual > 0.001:
                        omega = np.clip(dtheta / dt_actual, -self.w_max, self.w_max)
                    else:
                        omega = np.clip(dtheta / dt, -self.w_max, self.w_max)
                else:
                    omega = 0.0
            else:
                omega = 0.0
            
            trajectory.append({
                'x': x, 'y': y, 'theta': theta,
                'v': v, 'omega': omega, 't': t
            })
            
            # 时间步进 (基于实际距离和速度)
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
                'v': 0.0, 'omega': 0.0, 't': t
            })
        
        return trajectory
    
    def _normalize_angle(self, angle):
        """角度归一化到 [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    # ==================== 轨迹查询 ====================
    
    def get_reference_at_time(self, trajectory, t):
        """
        获取指定时间的参考状态 (线性插值)
        
        Args:
            trajectory: 轨迹
            t: 时间 (s)
            
        Returns:
            参考状态 dict 或 None
        """
        if not trajectory:
            return None
        
        for i in range(len(trajectory) - 1):
            if trajectory[i]['t'] <= t < trajectory[i+1]['t']:
                t0, t1 = trajectory[i]['t'], trajectory[i+1]['t']
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0
                
                ref = {}
                for key in ['x', 'y', 'theta', 'v', 'omega']:
                    ref[key] = trajectory[i][key] * (1 - alpha) + trajectory[i+1][key] * alpha
                ref['t'] = t
                return ref
        
        return trajectory[-1]
    
    def get_reference_window(self, trajectory, t_start, horizon, dt):
        """
        获取参考轨迹窗口 (用于 MPC)
        
        Args:
            trajectory: 完整轨迹
            t_start: 起始时间 (s)
            horizon: 预测步数
            dt: 时间步长 (s)
            
        Returns:
            参考点列表 [{...}, ...]
        """
        refs = []
        for i in range(horizon + 1):
            t = t_start + i * dt
            ref = self.get_reference_at_time(trajectory, t)
            if ref:
                refs.append(ref)
            elif refs:
                refs.append(refs[-1])
        return refs


# ==================== 测试代码 ====================

def _test():
    """单元测试"""
    print("=" * 50)
    print("TrajectoryGenerator 测试")
    print("=" * 50)
    
    # 创建测试路径
    path = [(0, 0), (1, 0), (2, 0.5), (3, 1), (3, 2)]
    current_state = np.array([0, 0, 0])
    
    generator = TrajectoryGenerator(v_max=0.2, v_min=0.05, w_max=1.5)
    traj = generator.generate(path, current_state, dt=0.1)
    
    if traj:
        print(f"✓ 轨迹生成成功: {len(traj)} 点")
        print(f"  总时长: {traj[-1]['t']:.2f}s")
        print(f"  起点: ({traj[0]['x']:.2f}, {traj[0]['y']:.2f})")
        print(f"  终点: ({traj[-1]['x']:.2f}, {traj[-1]['y']:.2f})")
        
        # 测试时间查询
        ref = generator.get_reference_at_time(traj, 5.0)
        if ref:
            print(f"  t=5.0s 状态: ({ref['x']:.2f}, {ref['y']:.2f}), v={ref['v']:.2f}")
        
        # 测试窗口查询
        window = generator.get_reference_window(traj, 0, 10, 0.1)
        print(f"  参考窗口: {len(window)} 点")
    else:
        print("✗ 轨迹生成失败")
    
    print("=" * 50)


if __name__ == '__main__':
    _test()
