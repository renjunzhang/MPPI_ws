#!/usr/bin/env python3
"""
路径处理模块 (Path Handler) - 参考 Nav2 MPPI 架构 + iLQR 轨迹平滑
================================================================

功能:
    - B-Spline 轨迹平滑 (使用 TrajectoryGenerator)
    - 路径剪裁 (Pruning): 移除机器人已经过的路径点
    - 动态前瞻 (Dynamic Lookahead): 根据速度调整前瞻距离
    - 基于时间的参考窗口提取

架构:
    A* 路径 (粗糙) → TrajectoryGenerator (B-Spline 平滑 + 速度规划) → PathHandler (剪裁) → MPPI

作者: GitHub Copilot
"""

import numpy as np

try:
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# 导入轨迹生成器 (B-Spline 平滑)
try:
    from local_planner import TrajectoryGenerator
    HAS_TRAJ_GEN = True
except ImportError:
    HAS_TRAJ_GEN = False


def _log_info(msg):
    if HAS_ROS and not rospy.is_shutdown():
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")


class PathHandler:
    """
    路径处理器 - Nav2 风格 + iLQR 轨迹平滑

    处理流程:
        1. 接收 A* 几何路径
        2. 使用 TrajectoryGenerator 进行 B-Spline 平滑 + 速度规划
        3. 路径剪裁 (Pruning): 移除已过路径点
        4. 提取参考窗口供 MPPI 使用
    """

    def __init__(self,
                 lookahead_dist=0.5,
                 min_lookahead=0.3,
                 max_lookahead=2.0,
                 prune_distance=0.3,
                 v_max=0.20,
                 v_min=0.05,
                 w_max=1.5,
                 a_max=0.5):
        """
        Args:
            lookahead_dist: 基础前瞻距离 (m)
            min_lookahead: 最小前瞻距离 (m)
            max_lookahead: 最大前瞻距离 (m)
            prune_distance: 剪裁距离阈值 (m)
            v_max, v_min, w_max, a_max: 速度/加速度约束
        """
        self.lookahead_dist = lookahead_dist
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.prune_distance = prune_distance

        # 轨迹生成器 (B-Spline 平滑 + 速度规划)
        if HAS_TRAJ_GEN:
            self.traj_generator = TrajectoryGenerator(
                v_max=v_max,
                v_min=v_min,
                w_max=w_max,
                a_max=a_max
            )
            _log_info(f"[PathHandler] TrajectoryGenerator enabled (v_max={v_max})")
        else:
            self.traj_generator = None
            _log_info("[PathHandler] TrajectoryGenerator not available, using raw interpolation")

        # 平滑后的轨迹 (带时间参数)
        self.smooth_trajectory = []
        # 活跃轨迹 (剪裁后)
        self.active_path = []
        # 原始几何路径
        self.geometric_path = []
        # 上一次剪裁的索引
        self.last_prune_idx = 0

        _log_info(f"[PathHandler] Init: lookahead={lookahead_dist}m, prune_dist={prune_distance}m")

    def set_global_path(self, path, current_state=None, dt=0.1):
        """
        设置新的全局路径 (来自 A*)

        Args:
            path: A* 几何路径 [(x, y), ...]
            current_state: 当前机器人状态 [x, y, theta] (用于轨迹生成)
            dt: 时间步长
        """
        if not path:
            self.geometric_path = []
            self.smooth_trajectory = []
            self.active_path = []
            self.last_prune_idx = 0
            return

        self.geometric_path = list(path)

        # === 关键: 使用 TrajectoryGenerator 进行 B-Spline 平滑 ===
        if self.traj_generator is not None and current_state is not None:
            traj = self.traj_generator.generate(path, current_state, dt=dt)
            if traj and len(traj) > 0:
                self.smooth_trajectory = traj
                self.active_path = list(traj)
                _log_info(f"[PathHandler] Smoothed: {len(path)} waypoints -> {len(traj)} trajectory points")
            else:
                # 平滑失败，退化为简单插值
                self._fallback_interpolate(path)
        else:
            # 没有 TrajectoryGenerator，使用简单插值
            self._fallback_interpolate(path)

        self.last_prune_idx = 0

    def _fallback_interpolate(self, path):
        """备用: 简单线性插值 (当没有 TrajectoryGenerator 时)"""
        if len(path) < 2:
            self.smooth_trajectory = []
            self.active_path = []
            return

        dense = []
        spacing = 0.05  # 5cm 间距

        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            x1, y1 = p1[0], p1[1]
            x2, y2 = p2[0], p2[1]
            dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            n_interp = max(1, int(dist / spacing))

            for j in range(n_interp):
                alpha = j / n_interp
                x = x1 + alpha * (x2 - x1)
                y = y1 + alpha * (y2 - y1)

                # 计算朝向
                theta = np.arctan2(y2 - y1, x2 - x1)

                dense.append({
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'v': 0.15,
                    'omega': 0.0
                })

        # 添加最后一个点
        dense.append({
            'x': path[-1][0],
            'y': path[-1][1],
            'theta': dense[-1]['theta'] if dense else 0.0,
            'v': 0.0,
            'omega': 0.0
        })

        self.smooth_trajectory = dense
        self.active_path = list(dense)
        _log_info(f"[PathHandler] Fallback interpolation: {len(path)} -> {len(dense)} points")

    def update(self, robot_pose, robot_speed=0.15):
        """
        更新路径处理器状态 (每控制周期调用)

        核心操作: 路径剪裁 (Pruning)

        Args:
            robot_pose: (x, y, theta) 机器人当前位姿
            robot_speed: 当前速度 (m/s)
        """
        if not self.active_path:
            return

        rx, ry = robot_pose[0], robot_pose[1]

        # === 修正2: 限制剪裁搜索范围 (防止 U 型/8 字型路径误剪) ===
        # 只搜索前 50 个点或前 2 米，避免在重叠/交叉路径上误判
        search_horizon = min(len(self.active_path), 50)

        min_dist = float('inf')
        closest_idx = 0

        for i in range(search_horizon):
            pt = self.active_path[i]
            dist = np.sqrt((pt['x'] - rx)**2 + (pt['y'] - ry)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 只剪裁到最近点之前的点
        if closest_idx > 0 and min_dist < self.prune_distance:
            self.active_path = self.active_path[closest_idx:]
            self.last_prune_idx += closest_idx

    def get_reference_window(self, robot_pose, horizon, dt, v_ref=0.15):
        """
        获取参考轨迹窗口 (MPPI 使用)

        关键改进:
            1. 修正1: 信任 B-Spline 计算的精准 theta (不重新差分计算)
            2. 修正3: 使用 lookahead_dist 动态计算起始点
            3. 基于时间步长采样

        Args:
            robot_pose: (x, y, theta)
            horizon: 预测步数 N
            dt: 时间步长
            v_ref: 参考速度 (备用)

        Returns:
            参考窗口 [{'x', 'y', 'theta', 'v'}, ...]
        """
        if not self.active_path:
            return []

        rx, ry, rtheta = robot_pose

        # 找到最近点 (也限制搜索范围)
        search_horizon = min(len(self.active_path), 50)
        min_dist = float('inf')
        closest_idx = 0
        for i in range(search_horizon):
            pt = self.active_path[i]
            dist = np.sqrt((pt['x'] - rx)**2 + (pt['y'] - ry)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # === 修正3: 使用 lookahead_dist 动态计算起始点 ===
        # 从最近点向前搜索，直到累积距离 >= lookahead_dist
        start_idx = closest_idx
        accumulated_dist = 0.0

        while start_idx < len(self.active_path) - 1:
            p1 = self.active_path[start_idx]
            p2 = self.active_path[start_idx + 1]
            seg_len = np.sqrt((p1['x']-p2['x'])**2 + (p1['y']-p2['y'])**2)
            accumulated_dist += seg_len
            start_idx += 1
            if accumulated_dist >= self.lookahead_dist:
                break

        # === 基于时间步长采样 ===
        window = []
        curr_idx = start_idx

        for k in range(horizon + 1):
            if curr_idx >= len(self.active_path):
                curr_idx = len(self.active_path) - 1

            pt = self.active_path[curr_idx]

            # === 修正1: 信任 B-Spline 计算的精准 theta ===
            # 只有第一步且离机器人较远时，才用差分计算引导方向
            if k == 0:
                dx = pt['x'] - rx
                dy = pt['y'] - ry
                dist_sq = dx*dx + dy*dy
                if dist_sq > 0.05 * 0.05:  # 距离 > 5cm 时引导切入
                    ref_theta = np.arctan2(dy, dx)
                else:
                    ref_theta = pt['theta']  # 离得近就信任路径角度
            else:
                # 后续点绝对信任 B-Spline 计算的精准角度
                ref_theta = pt['theta']

            # 使用平滑轨迹中的速度，或备用速度
            v = pt.get('v', v_ref)
            v = max(v, 0.08)  # 确保有最小速度

            window.append({
                'x': pt['x'],
                'y': pt['y'],
                'theta': ref_theta,
                'v': v
            })

            # 前进一个时间步的距离
            dist_step = v * dt
            dist_accumulated = 0.0

            while curr_idx < len(self.active_path) - 1:
                p1 = self.active_path[curr_idx]
                p2 = self.active_path[curr_idx + 1]
                seg_len = np.sqrt((p2['x']-p1['x'])**2 + (p2['y']-p1['y'])**2)

                if dist_accumulated + seg_len >= dist_step:
                    curr_idx += 1
                    break

                dist_accumulated += seg_len
                curr_idx += 1

        return window

    def get_full_active_path(self):
        """获取完整的活跃路径 (用于可视化)"""
        return self.active_path

    def get_smooth_trajectory(self):
        """获取平滑后的完整轨迹"""
        return self.smooth_trajectory

    def is_path_finished(self, robot_pose, threshold=0.2):
        """检查是否到达路径终点"""
        if not self.active_path:
            return True

        last_pt = self.active_path[-1]
        dist = np.sqrt((last_pt['x'] - robot_pose[0])**2 +
                       (last_pt['y'] - robot_pose[1])**2)
        return dist < threshold


# ==================== 测试 ====================

def _test():
    print("=" * 50)
    print("PathHandler 测试 (带 B-Spline 平滑)")
    print("=" * 50)

    handler = PathHandler(
        lookahead_dist=0.3,
        prune_distance=0.2,
        v_max=0.20,
        v_min=0.05
    )

    # 设置路径
    path = [(0, 0), (1, 0), (2, 0.5), (3, 1)]
    current_state = np.array([0, 0, 0])
    handler.set_global_path(path, current_state, dt=0.1)

    print(f"原始路径: {len(path)} 点")
    print(f"平滑后: {len(handler.smooth_trajectory)} 点")
    print(f"活跃路径: {len(handler.active_path)} 点")

    # 模拟机器人移动
    robot_poses = [
        (0.1, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.5, 0.2, 0.3),
    ]

    for pose in robot_poses:
        handler.update(pose)
        window = handler.get_reference_window(pose, horizon=10, dt=0.1)
        print(f"Robot at ({pose[0]:.1f}, {pose[1]:.1f}): "
              f"active={len(handler.active_path)}, window={len(window)}")

    print("=" * 50)


if __name__ == '__main__':
    _test()
