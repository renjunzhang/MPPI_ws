#!/usr/bin/env python3
"""
二层规划控制架构 - MPPI 控制器 (Nav2 风格改进版)
==============================================

架构:
    Layer 1: 全局路径规划 (A*) - global_planner.py
    Layer 2+3: MPPI (轨迹优化 + 控制一体化)

改进点 (参考 Nav2 MPPI):
    1. PathHandler: 路径剪裁，防止回头
    2. Costmap 查表: 代替障碍物点列表
    3. 模块化 Critics: 代价函数拆分
    4. 软约束: 代替硬约束

作者: GitHub Copilot
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

# 导入模块
from global_planner import GlobalPathPlanner
from path_handler import PathHandler
from mppi_node import MPPITracker


class MPPITwoLayerController:
    """
    二层规划控制器 (A* + MPPI) - Nav2 风格

    改进:
        - PathHandler 实现路径剪裁
        - Costmap 直接传给 MPPI
        - 模块化 Critics
    """

    def __init__(self):
        rospy.init_node('mppi_two_layer_controller')

        self._print_banner()

        # 状态变量
        self.goal_pos = None
        self.current_state = None
        self.obstacle_points = []
        self.map_received = False

        # 性能指标
        self._metrics_poses = []
        self._metrics_speeds = []
        self._metrics_ctes = []

        # ===== 参数 =====
        self.dt = 0.1
        self.N = 30  # MPPI 预测步数
        self.replan_interval = 3.0
        self.last_replan_time = 0.0
        self.v_ref = rospy.get_param("~v_ref", 0.15)  # 参考速度

        # ===== Layer 1: 全局规划器 =====
        self.global_planner = GlobalPathPlanner(resolution=0.05, inflate_radius=0.25)

        # ===== PathHandler: 路径处理 (Nav2 风格 + B-Spline 平滑) =====
        self.path_handler = PathHandler(
            lookahead_dist=rospy.get_param("~lookahead_dist", 0.3),
            prune_distance=rospy.get_param("~prune_distance", 0.2),
            v_max=0.20,
            v_min=0.05,
            w_max=1.5,
            a_max=0.5
        )

        # ===== Layer 2+3: MPPI 控制器 =====
        self.mppi_tracker = MPPITracker(dt=self.dt, N=self.N)

        # 规划结果
        self.geometric_path = []

        # ROS 接口
        self._setup_ros()

        rospy.loginfo("[MPPIController] Ready (Nav2 Style)")

    def _print_banner(self):
        """打印启动信息"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("Two-Layer Controller (A* + MPPI) - Nav2 Style")
        rospy.loginfo("  L1: Global Path Planning (A*)")
        rospy.loginfo("  L2+3: MPPI (Trajectory Optimization + Control)")
        rospy.loginfo("  Features: PathHandler, Costmap, Modular Critics")
        rospy.loginfo("=" * 50)

    def _setup_ros(self):
        """初始化 ROS 接口"""
        # Publishers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_active_path = rospy.Publisher('/active_path', Path, queue_size=1)
        self.pub_mppi_traj = rospy.Publisher('/mppi_predict_path', Path, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.pub_metrics = rospy.Publisher('/nav_metrics', Float32MultiArray, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/custom_goal', PoseStamped, self._goal_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        rospy.Subscriber('/map', OccupancyGrid, self._map_cb)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(self.dt), self._control_loop)

    # ==================== 回调函数 ====================

    def _odom_cb(self, msg):
        """里程计回调"""
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state = np.array([pos.x, pos.y, yaw])

    def _goal_cb(self, msg):
        """目标点回调"""
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.geometric_path = []
        self.path_handler.set_global_path([])  # 清空路径
        self.last_replan_time = 0
        self.mppi_tracker.reset()

        # 重置性能指标
        self._metrics_poses = []
        self._metrics_speeds = []
        self._metrics_ctes = []

        rospy.loginfo(f"[MPPIController] New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")

    def _scan_cb(self, msg):
        """激光雷达回调 - 提取动态障碍物"""
        if self.current_state is None:
            return

        x, y, theta = self.current_state
        obstacles = []
        angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            if i % 3 == 0 and msg.range_min < r < msg.range_max and r < 3.0:
                ox = x + r * np.cos(theta + angle)
                oy = y + r * np.sin(theta + angle)
                obstacles.append((ox, oy))
            angle += msg.angle_increment

        self.obstacle_points = obstacles

    def _map_cb(self, msg):
        """地图回调"""
        if not self.map_received:
            rospy.loginfo(f"[MPPIController] Map: {msg.info.width}x{msg.info.height}")

        self.global_planner.load_static_map(msg)
        self.map_received = True

        # 将 Costmap 传给 MPPI (Nav2 风格)
        if self.global_planner.grid is not None:
            self.mppi_tracker.set_costmap(
                self.global_planner.grid,
                self.global_planner.resolution,
                (self.global_planner.origin_x, self.global_planner.origin_y)
            )

    # ==================== 控制循环 ====================

    def _control_loop(self, event):
        """主控制循环"""
        if self.current_state is None:
            return

        # 无目标时停止
        if self.goal_pos is None:
            self.pub_cmd.publish(Twist())
            return

        # 检查是否到达目标
        dist = np.linalg.norm(self.goal_pos - self.current_state[:2])
        if dist < 0.15:
            self._publish_metrics()
            rospy.loginfo("[MPPIController] Goal reached!")
            self.goal_pos = None
            self.geometric_path = []
            self.pub_cmd.publish(Twist())
            return

        # 定期重规划全局路径
        current_time = rospy.Time.now().to_sec()
        need_replan = (current_time - self.last_replan_time > self.replan_interval or
                       len(self.geometric_path) == 0)

        if need_replan:
            self._do_global_planning()
            self.last_replan_time = current_time

        # 检查路径是否有效
        if self.path_handler.is_path_finished(self.current_state):
            self.pub_cmd.publish(Twist())
            return

        # === 关键: PathHandler 更新 (路径剪裁) ===
        self.path_handler.update(self.current_state)

        # 获取参考窗口
        ref_window = self.path_handler.get_reference_window(
            self.current_state,
            horizon=self.N,
            dt=self.dt,
            v_ref=self.v_ref
        )

        if len(ref_window) == 0:
            self.pub_cmd.publish(Twist())
            return

        # MPPI 求解 (使用 Costmap，不用障碍物点列表)
        control, mppi_traj = self.mppi_tracker.solve(
            self.current_state,
            ref_window,
            obstacles_list=self.obstacle_points  # 仍传入作为备用
        )

        # 发布控制
        cmd = Twist()
        cmd.linear.x = float(control[0])
        cmd.angular.z = float(control[1])
        self.pub_cmd.publish(cmd)

        # 记录性能指标
        self._record_metrics(control)

        # 可视化
        self._visualize(mppi_traj)

        rospy.loginfo_throttle(1.0,
            f"[MPPIController] dist={dist:.2f}m, v={control[0]:.2f}, w={control[1]:.2f}, "
            f"active_path={len(self.path_handler.active_path)}")

    def _do_global_planning(self):
        """执行全局规划 (Layer 1)"""
        if not self.map_received or self.goal_pos is None:
            return

        # Layer 1: A* 全局规划
        self.global_planner.update_dynamic_obstacles(self.obstacle_points)
        path = self.global_planner.plan(self.current_state[:2], self.goal_pos)

        if path is None:
            rospy.logwarn("[MPPIController] Global planning failed!")
            return

        self.geometric_path = path

        # === 关键: 将路径传给 PathHandler (使用 B-Spline 平滑) ===
        self.path_handler.set_global_path(path, self.current_state, dt=self.dt)

        rospy.loginfo(f"[MPPIController] Plan: {len(path)} waypoints -> "
                      f"{len(self.path_handler.active_path)} smooth trajectory points")

    def _record_metrics(self, control):
        """记录性能指标"""
        self._metrics_poses.append((self.current_state[0], self.current_state[1]))
        self._metrics_speeds.append(float(control[0]))

        # CTE: 到活跃路径最近点的距离
        active_path = self.path_handler.get_full_active_path()
        if active_path:
            min_dist = min(
                np.sqrt((self.current_state[0]-p['x'])**2 +
                        (self.current_state[1]-p['y'])**2)
                for p in active_path
            )
            if min_dist < 0.5:
                self._metrics_ctes.append(min_dist)

    # ==================== 可视化 ====================

    def _visualize(self, mppi_traj):
        """发布可视化消息"""
        now = rospy.Time.now()

        # 全局路径 (绿色)
        if self.geometric_path:
            self._publish_path(self.pub_global_path, self.geometric_path, now)

        # 活跃路径 (蓝色) - 剪裁后的路径
        active_path = self.path_handler.get_full_active_path()
        if active_path:
            pts = [(p['x'], p['y']) for p in active_path]
            self._publish_path(self.pub_active_path, pts, now)

        # MPPI 预测轨迹 (红色)
        if mppi_traj is not None:
            pts = [(s[0], s[1]) for s in mppi_traj]
            self._publish_path(self.pub_mppi_traj, pts, now)

        # 目标点
        if self.goal_pos is not None:
            self._publish_goal_marker(now)

    def _publish_path(self, pub, points, stamp):
        """发布路径"""
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

    def _publish_goal_marker(self, stamp):
        """发布目标点标记"""
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = stamp
        m.ns = "goal"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.goal_pos[0]
        m.pose.position.y = self.goal_pos[1]
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.3
        m.scale.z = 0.2
        m.color.a = 0.8
        m.color.r = 1.0
        m.color.g = 0.5
        self.pub_goal_marker.publish(m)

    def _publish_metrics(self):
        """导航结束时计算并发布性能指标"""
        if len(self._metrics_poses) < 2:
            return

        # 路径长度
        length = sum(
            np.sqrt((self._metrics_poses[i+1][0]-self._metrics_poses[i][0])**2 +
                    (self._metrics_poses[i+1][1]-self._metrics_poses[i][1])**2)
            for i in range(len(self._metrics_poses)-1)
        )
        # 平均速度
        avg_speed = np.mean(self._metrics_speeds) if self._metrics_speeds else 0.0
        # 平均/最大横向误差
        avg_cte = np.mean(self._metrics_ctes) if self._metrics_ctes else 0.0
        max_cte = np.max(self._metrics_ctes) if self._metrics_ctes else 0.0

        # 发布
        msg = Float32MultiArray()
        msg.data = [float(length), float(avg_speed), float(avg_cte), float(max_cte)]
        self.pub_metrics.publish(msg)

        rospy.loginfo(f"[Metrics] 路径长度:{length:.2f}m, 平均速度:{avg_speed:.2f}m/s, "
                      f"平均CTE:{avg_cte:.3f}m, 最大CTE:{max_cte:.3f}m")


if __name__ == '__main__':
    try:
        MPPITwoLayerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
