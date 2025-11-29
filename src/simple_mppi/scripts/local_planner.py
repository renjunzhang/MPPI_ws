#!/usr/bin/env python3
"""
局部轨迹规划模块 (Layer 2)
==========================
将几何路径转换为时间参数化轨迹

功能:
    - 路径插值 (密集采样)
    - 曲率计算
    - 基于曲率的速度规划 (弯道减速)
    - 加减速处理
    - 时间参数化轨迹生成

输入: 几何路径 [(x, y), ...], 当前状态 [x, y, theta]
输出: 时间参数化轨迹 [{x, y, theta, v, omega, t}, ...]

作者: GitHub Copilot
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


class TrajectoryGenerator:
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
    
    def __init__(self, v_max=0.20, v_min=0.05, w_max=1.5):
        """
        初始化轨迹生成器
        
        Args:
            v_max: 最大线速度 (m/s)
            v_min: 最小线速度 (m/s)
            w_max: 最大角速度 (rad/s)
        """
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        
        _log_info("[TrajectoryGenerator] Initialized")
    
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
        
        # Step 1: 路径插值
        dense_path = self._interpolate_path(geometric_path, spacing=0.05)
        
        # Step 2: 计算曲率
        curvatures = self._compute_curvatures(dense_path)
        
        # Step 3: 速度规划
        speeds = self._plan_speeds(dense_path, curvatures)
        
        # Step 4: 生成轨迹
        trajectory = self._build_trajectory(dense_path, speeds, current_state, dt)
        
        # Step 5: 添加停止段
        trajectory = self._add_stop_segment(trajectory, dt)
        
        total_time = trajectory[-1]['t'] if trajectory else 0
        _log_info(f"[TrajectoryGenerator] Trajectory: {len(trajectory)} pts, {total_time:.1f}s")
        
        return trajectory
    
    def _interpolate_path(self, path, spacing=0.05):
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
    
    def _plan_speeds(self, path, curvatures):
        """
        基于曲率的速度规划
        
        - 直道高速
        - 弯道减速
        - 起点加速
        - 终点减速
        """
        n = len(path)
        speeds = [self.v_max] * n
        
        # 1. 曲率自适应 (弯道减速)
        curvature_sensitivity = 2.0
        for i in range(n):
            speeds[i] = self.v_max / (1 + curvature_sensitivity * curvatures[i])
            speeds[i] = np.clip(speeds[i], self.v_min, self.v_max)
        
        # 2. 终点减速
        decel_dist = 0.3
        total_dist = 0
        for i in range(n - 1, 0, -1):
            dist = np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            total_dist += dist
            if total_dist < decel_dist:
                speeds[i] = min(speeds[i], self.v_max * total_dist / decel_dist)
                speeds[i] = max(0.0, speeds[i])
        
        # 最后几个点速度为0
        for i in range(max(0, n-3), n):
            speeds[i] = 0.0
        
        # 3. 起点加速
        accel_dist = 0.2
        total_dist = 0
        for i in range(n - 1):
            dist = np.sqrt((path[i+1][0]-path[i][0])**2 + (path[i+1][1]-path[i][1])**2)
            total_dist += dist
            if total_dist < accel_dist:
                speeds[i] = min(speeds[i], self.v_max * total_dist / accel_dist + self.v_min)
        
        return speeds
    
    def _build_trajectory(self, path, speeds, current_state, dt):
        """构建轨迹"""
        trajectory = []
        t = 0.0
        
        for i in range(len(path)):
            x, y = path[i]
            v = speeds[i]
            
            # 计算朝向
            if i < len(path) - 1:
                dx = path[i+1][0] - x
                dy = path[i+1][1] - y
                theta = np.arctan2(dy, dx)
            else:
                theta = trajectory[-1]['theta'] if trajectory else current_state[2]
            
            # 计算角速度
            if i > 0:
                dtheta = self._normalize_angle(theta - trajectory[-1]['theta'])
                omega = np.clip(dtheta / dt, -self.w_max, self.w_max) if dt > 0 else 0.0
            else:
                omega = 0.0
            
            trajectory.append({
                'x': x, 'y': y, 'theta': theta,
                'v': v, 'omega': omega, 't': t
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
