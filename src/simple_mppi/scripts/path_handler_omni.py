#!/usr/bin/env python3
"""
路径处理模块 (Path Handler) - 全向移动版本
==========================================

与差速版本的区别:
    - 参考轨迹包含 vx, vy 两个速度分量
    - 支持横向速度规划
    - 朝向可以独立于运动方向

作者: GitHub Copilot
"""

import numpy as np

try:
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# 导入轨迹生成器
try:
    from local_planner_omni import TrajectoryGeneratorOmni
    HAS_TRAJ_GEN = True
except ImportError:
    HAS_TRAJ_GEN = False


def _log_info(msg):
    if HAS_ROS and not rospy.is_shutdown():
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")


class PathHandlerOmni:
    """
    路径处理器 - 全向移动版本

    特点:
        - 支持 vx, vy 两个速度分量
        - 轨迹点格式: {'x', 'y', 'theta', 'vx', 'vy'}
    """

    def __init__(self,
                 lookahead_dist=0.5,
                 min_lookahead=0.3,
                 max_lookahead=2.0,
                 prune_distance=0.3,
                 v_max=0.5,
                 v_min=0.05,
                 w_max=1.0,
                 a_max=0.8):
        """
        Args:
            lookahead_dist: 基础前瞻距离 (m)
            min_lookahead: 最小前瞻距离 (m)
            max_lookahead: 最大前瞻距离 (m)
            prune_distance: 剪裁距离阈值 (m)
            v_max, v_min, w_max, a_max: 约束
        """
        self.lookahead_dist = lookahead_dist
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.prune_distance = prune_distance
        self.v_max = v_max
        self.v_min = v_min

        # 轨迹生成器 (全向版本)
        if HAS_TRAJ_GEN:
            self.traj_generator = TrajectoryGeneratorOmni(
                v_max=v_max,
                v_min=v_min,
                w_max=w_max,
                a_max=a_max
            )
            _log_info(f"[PathHandlerOmni] TrajectoryGeneratorOmni enabled (v_max={v_max})")
        else:
            self.traj_generator = None
            _log_info("[PathHandlerOmni] TrajectoryGenerator not available, using fallback")

        # 轨迹数据
        self.smooth_trajectory = []
        self.active_path = []
        self.geometric_path = []
        self.last_prune_idx = 0

        _log_info(f"[PathHandlerOmni] Init: lookahead={lookahead_dist}m, prune_dist={prune_distance}m")

    def set_global_path(self, path, current_state=None, dt=0.1):
        """
        设置新的全局路径

        Args:
            path: A* 几何路径 [(x, y), ...]
            current_state: [x, y, theta]
            dt: 时间步长
        """
        if not path:
            self.geometric_path = []
            self.smooth_trajectory = []
            self.active_path = []
            self.last_prune_idx = 0
            return

        self.geometric_path = list(path)

        # 使用轨迹生成器平滑
        if self.traj_generator is not None and current_state is not None:
            traj = self.traj_generator.generate(path, current_state, dt=dt)
            if traj and len(traj) > 0:
                self.smooth_trajectory = traj
                self.active_path = list(traj)
                _log_info(f"[PathHandlerOmni] Smoothed: {len(path)} -> {len(traj)} points")
            else:
                self._fallback_interpolate(path)
        else:
            self._fallback_interpolate(path)

        self.last_prune_idx = 0

    def _fallback_interpolate(self, path):
        """备用: 简单线性插值"""
        if len(path) < 2:
            self.smooth_trajectory = []
            self.active_path = []
            return

        dense = []
        spacing = 0.05

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

                # 计算朝向 (沿路径方向)
                theta = np.arctan2(y2 - y1, x2 - x1)

                # 全向: 主要使用前向速度
                dense.append({
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'vx': 0.2,  # 前向速度
                    'vy': 0.0,  # 横向速度
                    'v': 0.2,   # 兼容旧格式
                    'omega': 0.0
                })

        # 最后一个点
        dense.append({
            'x': path[-1][0],
            'y': path[-1][1],
            'theta': dense[-1]['theta'] if dense else 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'v': 0.0,
            'omega': 0.0
        })

        self.smooth_trajectory = dense
        self.active_path = list(dense)
        _log_info(f"[PathHandlerOmni] Fallback: {len(path)} -> {len(dense)} points")

    def update(self, robot_pose, robot_speed=0.2):
        """
        更新路径处理器 (路径剪裁)

        Args:
            robot_pose: (x, y, theta)
            robot_speed: 当前速度
        """
        if not self.active_path:
            return

        rx, ry = robot_pose[0], robot_pose[1]

        # 限制搜索范围
        search_horizon = min(len(self.active_path), 50)

        min_dist = float('inf')
        closest_idx = 0

        for i in range(search_horizon):
            pt = self.active_path[i]
            dist = np.sqrt((pt['x'] - rx)**2 + (pt['y'] - ry)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 剪裁
        if closest_idx > 0 and min_dist < self.prune_distance:
            self.active_path = self.active_path[closest_idx:]
            self.last_prune_idx += closest_idx

    def get_reference_window(self, robot_pose, horizon, dt, v_ref=0.3):
        """
        获取参考轨迹窗口 - 空间驱动版本

        Args:
            robot_pose: (x, y, theta)
            horizon: 预测步数
            dt: 时间步长
            v_ref: 参考速度

        Returns:
            参考窗口 [{'x', 'y', 'theta', 'vx', 'vy'}, ...]
        """
        if not self.active_path:
            return []

        rx, ry, rtheta = robot_pose

        # 找最近点
        search_horizon = min(len(self.active_path), 50)
        min_dist = float('inf')
        closest_idx = 0

        for i in range(search_horizon):
            pt = self.active_path[i]
            dist = np.sqrt((pt['x'] - rx)**2 + (pt['y'] - ry)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 从最近点开始前瞻
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

        # 截取 horizon 个点
        window = []
        curr_idx = start_idx

        for k in range(horizon + 1):
            if curr_idx >= len(self.active_path):
                curr_idx = len(self.active_path) - 1

            pt = self.active_path[curr_idx]

            # 第一步: 引导切入
            if k == 0:
                dx = pt['x'] - rx
                dy = pt['y'] - ry
                dist_sq = dx*dx + dy*dy
                if dist_sq > 0.05 * 0.05:
                    ref_theta = np.arctan2(dy, dx)
                else:
                    ref_theta = pt['theta']
            else:
                ref_theta = pt['theta']

            # 获取速度 (全向)
            vx = pt.get('vx', pt.get('v', v_ref))
            vy = pt.get('vy', 0.0)
            vx = max(vx, 0.08)

            window.append({
                'x': pt['x'],
                'y': pt['y'],
                'theta': ref_theta,
                'vx': vx,
                'vy': vy,
                'v': vx  # 兼容
            })

            # 前进一步
            dist_step = vx * dt
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
        """获取完整活跃路径"""
        return self.active_path

    def get_smooth_trajectory(self):
        """获取平滑轨迹"""
        return self.smooth_trajectory

    def is_path_finished(self, robot_pose, threshold=0.25):
        """检查是否到达终点"""
        if not self.active_path:
            return True

        last_pt = self.active_path[-1]
        dist = np.sqrt((last_pt['x'] - robot_pose[0])**2 +
                       (last_pt['y'] - robot_pose[1])**2)
        return dist < threshold


# ==================== 测试 ====================

def _test():
    print("=" * 50)
    print("PathHandlerOmni 测试")
    print("=" * 50)

    handler = PathHandlerOmni(
        lookahead_dist=0.3,
        prune_distance=0.2,
        v_max=0.5,
        v_min=0.05
    )

    path = [(0, 0), (1, 0), (2, 0.5), (3, 1)]
    current_state = np.array([0, 0, 0])
    handler.set_global_path(path, current_state, dt=0.1)

    print(f"原始路径: {len(path)} 点")
    print(f"活跃路径: {len(handler.active_path)} 点")

    # 测试参考窗口
    window = handler.get_reference_window(current_state, horizon=10, dt=0.1)
    print(f"参考窗口: {len(window)} 点")

    if window:
        print(f"  第一点: x={window[0]['x']:.2f}, vx={window[0]['vx']:.2f}, vy={window[0]['vy']:.2f}")

    print("=" * 50)


if __name__ == '__main__':
    _test()
