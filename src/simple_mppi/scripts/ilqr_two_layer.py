#!/usr/bin/env python3
"""
二层规划控制架构 - iLQR 控制器
==============================

架构:
    Layer 1: A* 全局规划 + B-Spline 平滑 + 速度规划
    Layer 2: iLQR 轨迹跟踪控制

模块职责:
    - global_planner.py: A* 路径搜索
    - local_planner.py: 轨迹平滑 + 速度规划
    - ilqr_tracker.py: iLQR 核心算法
    - 本文件: ROS 接口 + 模块整合
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

from global_planner import GlobalPathPlanner
from local_planner import TrajectoryGenerator
from ilqr_tracker import ILQRTracker


class ILQRTwoLayerController:
    """二层规划控制器 (A* + iLQR)"""

    def __init__(self):
        rospy.init_node('ilqr_two_layer_controller')
        rospy.loginfo("=" * 40)
        rospy.loginfo("Two-Layer Controller (A* + iLQR)")
        rospy.loginfo("=" * 40)

        # 参数
        self.dt = 0.1
        self.N = 20
        self.v_max = 0.22
        self.replan_interval = 3.0

        # 状态
        self.goal_pos = None
        self.current_state = None
        self.obstacle_points = []
        self.map_received = False
        self.last_replan_time = 0.0

        # 规划结果
        self.geometric_path = []
        self.smooth_ref = []

        # 模块初始化
        obs_eta = rospy.get_param("~obs_eta", 100.0)
        obs_d0 = rospy.get_param("~obs_d0", 0.6)

        self.global_planner = GlobalPathPlanner(resolution=0.05, inflate_radius=0.25)
        self.traj_generator = TrajectoryGenerator(v_max=self.v_max, v_min=0.05, w_max=2.0, a_max=0.5)
        self.ilqr = ILQRTracker(
            dt=self.dt, N=self.N, v_max=self.v_max, v_min=0.0, w_max=1.0,  # 降低 w_max 防止剧烈转向
            Q_pos=120.0,   # 位置权重（最重要）
            Q_theta=1.0,   # 大幅降低角度权重，让它自然跟随
            Q_v=2.0,       # 速度跟踪
            R_v=0.1,       # 控制平滑
            R_w=0.5,       # 提高角速度代价，防止来回转
            max_iter=10, reg=0.1, obs_eta=obs_eta, obs_d0=obs_d0
        )

        # ROS 接口
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_smooth_path = rospy.Publisher('/smooth_path', Path, queue_size=1)
        self.pub_ilqr_traj = rospy.Publisher('/ilqr_predict_path', Path, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)

        rospy.Subscriber('/custom_goal', PoseStamped, self._goal_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        rospy.Subscriber('/map', OccupancyGrid, self._map_cb)

        rospy.Timer(rospy.Duration(self.dt), self._control_loop)
        rospy.loginfo("[iLQRController] Ready")

    # ==================== 回调 ====================

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state = np.array([pos.x, pos.y, yaw])

    def _goal_cb(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.geometric_path = []
        self.smooth_ref = []
        self.last_replan_time = 0
        self.ilqr.reset()
        rospy.loginfo(f"[iLQR] New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")

    def _scan_cb(self, msg):
        if self.current_state is None:
            return
        x, y, theta = self.current_state
        obstacles = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 3 == 0 and msg.range_min < r < min(msg.range_max, 3.0):
                obstacles.append((x + r * np.cos(theta + angle), y + r * np.sin(theta + angle)))
            angle += msg.angle_increment
        self.obstacle_points = obstacles

    def _map_cb(self, msg):
        if not self.map_received:
            rospy.loginfo(f"[iLQR] Map: {msg.info.width}x{msg.info.height}")
        self.global_planner.load_static_map(msg)
        self.map_received = True

    # ==================== 控制 ====================

    def _control_loop(self, event):
        if self.current_state is None or self.goal_pos is None:
            return

        dist = np.linalg.norm(self.goal_pos - self.current_state[:2])
        if dist < 0.15:
            rospy.loginfo("[iLQR] Goal reached!")
            self.goal_pos = None
            self.pub_cmd.publish(Twist())
            return

        # 定期重规划
        now = rospy.Time.now().to_sec()
        if now - self.last_replan_time > self.replan_interval or len(self.smooth_ref) == 0:
            self._do_planning()
            self.last_replan_time = now

        if len(self.smooth_ref) == 0:
            self.pub_cmd.publish(Twist())
            return

        # 获取参考窗口
        ref_window = self._get_ref_window()
        if len(ref_window) == 0:
            self.pub_cmd.publish(Twist())
            return

        # iLQR 求解
        control, pred_traj = self.ilqr.solve(self.current_state, ref_window, self.obstacle_points)

        # 发布控制
        cmd = Twist()
        cmd.linear.x = float(control[0])
        cmd.angular.z = float(control[1])
        self.pub_cmd.publish(cmd)

        # 可视化
        self._visualize(pred_traj)
        rospy.loginfo_throttle(1.0, f"[iLQR] dist={dist:.2f}m, v={control[0]:.2f}, w={control[1]:.2f}")

    def _do_planning(self):
        """执行全局规划 + 轨迹生成"""
        if not self.map_received:
            return

        # Layer 1: A* 规划
        self.global_planner.update_dynamic_obstacles(self.obstacle_points)
        path = self.global_planner.plan(self.current_state[:2], self.goal_pos)
        if path is None:
            rospy.logwarn("[iLQR] Planning failed!")
            return

        self.geometric_path = path

        # Layer 1.5: 轨迹平滑 + 速度规划
        traj = self.traj_generator.generate(path, self.current_state, dt=self.dt)
        self.smooth_ref = traj if traj else []
        rospy.loginfo(f"[iLQR] Plan: {len(path)} waypoints -> {len(self.smooth_ref)} ref pts")

    def _get_ref_window(self):
        """
        基于时间重采样：将几何路径转换为符合 dt 的时间轨迹

        关键改进：
        1. 参考点的 theta 要和机器人当前朝向一致（从机器人指向参考点）
        2. 不使用路径自带的 theta，而是根据机器人到参考点的方向计算
        """
        if not self.smooth_ref:
            return []

        x, y, theta = self.current_state

        # 1. 找到路径上离当前位置最近的点（只搜索前方）
        min_dist = float('inf')
        closest_idx = 0

        for i in range(len(self.smooth_ref)):
            pt = self.smooth_ref[i]
            dx = pt['x'] - x
            dy = pt['y'] - y
            dist = np.sqrt(dx*dx + dy*dy)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 2. 从最近点往前跳几个点（前瞻）
        start_idx = min(closest_idx + 3, len(self.smooth_ref) - 1)

        # 3. 基于时间重采样生成参考窗口
        ref_window = []
        curr_idx = start_idx

        for k in range(self.N + 1):
            if curr_idx >= len(self.smooth_ref):
                curr_idx = len(self.smooth_ref) - 1

            pt = self.smooth_ref[curr_idx]

            # 关键：计算从当前位置指向参考点的方向作为参考 theta
            # 而不是使用路径自带的 theta
            if k == 0:
                # 第一个点：从机器人当前位置指向该点
                dx = pt['x'] - x
                dy = pt['y'] - y
                if dx*dx + dy*dy > 0.001:
                    ref_theta = np.arctan2(dy, dx)
                else:
                    ref_theta = theta  # 太近了，保持当前朝向
            else:
                # 后续点：从上一个参考点指向当前点
                prev_pt = ref_window[-1]
                dx = pt['x'] - prev_pt['x']
                dy = pt['y'] - prev_pt['y']
                if dx*dx + dy*dy > 0.001:
                    ref_theta = np.arctan2(dy, dx)
                else:
                    ref_theta = prev_pt['theta']

            # 确保有速度
            v_ref = max(pt.get('v', 0.15), 0.10)

            ref_window.append({
                'x': pt['x'],
                'y': pt['y'],
                'theta': ref_theta,
                'v': v_ref,
                'omega': pt.get('omega', 0.0)
            })

            # 计算这一步 dt 会走多远
            dist_step = v_ref * self.dt

            # 在几何路径上前进 dist_step 距离
            dist_accumulated = 0.0
            while curr_idx < len(self.smooth_ref) - 1:
                p1 = self.smooth_ref[curr_idx]
                p2 = self.smooth_ref[curr_idx + 1]
                seg_len = np.sqrt((p2['x']-p1['x'])**2 + (p2['y']-p1['y'])**2)

                if dist_accumulated + seg_len >= dist_step:
                    curr_idx += 1
                    break

                dist_accumulated += seg_len
                curr_idx += 1

        return ref_window

    # ==================== 可视化 ====================

    def _visualize(self, pred_traj):
        now = rospy.Time.now()

        if self.geometric_path:
            self._pub_path(self.pub_global_path, self.geometric_path, now)

        if self.smooth_ref:
            pts = [(p['x'], p['y']) for p in self.smooth_ref]
            self._pub_path(self.pub_smooth_path, pts, now)

        if pred_traj:
            pts = [(s[0], s[1]) for s in pred_traj]
            self._pub_path(self.pub_ilqr_traj, pts, now)

        if self.goal_pos is not None:
            self._pub_goal(now)

    def _pub_path(self, pub, points, stamp):
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = stamp
        for p in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        pub.publish(msg)

    def _pub_goal(self, stamp):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = stamp
        m.ns = "goal"
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.goal_pos[0]
        m.pose.position.y = self.goal_pos[1]
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.3
        m.scale.z = 0.2
        m.color.a = 0.8
        m.color.b = 1.0
        m.color.g = 0.5
        self.pub_goal_marker.publish(m)


if __name__ == '__main__':
    try:
        ILQRTwoLayerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
