#!/usr/bin/env python3
"""
二层规划控制架构 - 全向移动 MPPI 控制器 (Ridgeback)
====================================================

架构:
    Layer 1: 全局路径规划 (A*)
    Layer 2+3: MPPI (轨迹优化 + 控制一体化)

与差速版本的区别:
    - 使用 geometry_msgs/Twist 的 linear.x, linear.y, angular.z
    - 支持全向运动学模型
    - 适配 Ridgeback 话题

作者: GitHub Copilot
"""

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

# 导入模块
from global_planner import GlobalPathPlanner
from path_handler_omni import PathHandlerOmni
from mppi_omni import MPPIOmniTracker

# 尝试导入指标收集器
try:
    from metrics_collector import MetricsCollector
    HAS_METRICS = True
except ImportError:
    HAS_METRICS = False


class MPPIRidgebackController:
    """
    全向移动 MPPI 控制器 (Ridgeback)

    特点:
        - 3维控制输出 [vx, vy, omega]
        - 全向运动学模型
        - 适配 Ridgeback 仿真环境
    """

    def __init__(self):
        rospy.init_node('mppi_ridgeback_controller')

        self._print_banner()

        # 状态变量
        self.goal_pos = None
        self.goal_theta = None  # 目标朝向
        self.current_state = None
        self.obstacle_points = []
        self.map_received = False

        # 无地图模式: 当没有静态地图时，使用直线路径 + 激光避障
        self.mapless_mode = rospy.get_param("~mapless_mode", True)
        self.map_wait_time = 5.0  # 等待地图的时间 (秒)
        self.start_time = rospy.Time.now().to_sec()

        # 性能指标收集器
        if HAS_METRICS:
            self.metrics = MetricsCollector(algo_name="MPPI_Omni")
        else:
            self.metrics = None

        # ===== 参数 =====
        self.dt = 0.1
        self.N = 20
        self.replan_interval = 5.0
        self.last_replan_time = 0.0
        self.v_ref = rospy.get_param("~v_ref", 0.3)

        # ===== Layer 1: 全局规划器 =====
        self.global_planner = GlobalPathPlanner(
            resolution=0.05,
            inflate_radius=0.35  # Ridgeback 更大, 需要更大膨胀
        )

        # ===== PathHandler: 全向版本 =====
        self.path_handler = PathHandlerOmni(
            lookahead_dist=rospy.get_param("~lookahead_dist", 0.5),
            prune_distance=rospy.get_param("~prune_distance", 0.2),
            v_max=rospy.get_param("~vx_max", 0.5),
            v_min=0.05,
            w_max=rospy.get_param("~w_max", 1.0),
            a_max=0.8
        )

        # ===== Layer 2+3: MPPI 控制器 (全向版本) =====
        self.mppi_tracker = MPPIOmniTracker(dt=self.dt, N=self.N)

        # 规划结果
        self.geometric_path = []

        # 一阶低通滤波器 (cmd_vel)
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.3)  # 滤波系数 [0,1]
        self.last_cmd = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega]

        # ROS 接口
        self._setup_ros()

        rospy.loginfo("[MPPIRidgeback] Ready (Omnidirectional)")

    def _print_banner(self):
        """打印启动信息"""
        rospy.loginfo("=" * 55)
        rospy.loginfo("  Ridgeback Omnidirectional MPPI Controller")
        rospy.loginfo("  L1: Global Path Planning (A*)")
        rospy.loginfo("  L2+3: MPPI (3D Control: vx, vy, omega)")
        rospy.loginfo("=" * 55)

    def _setup_ros(self):
        """初始化 ROS 接口"""
        # Publishers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_active_path = rospy.Publisher('/active_path', Path, queue_size=1)
        self.pub_mppi_traj = rospy.Publisher('/mppi_predict_path', Path, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)

        # Subscribers (使用 remap 适配不同话题名)
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

        # 提取目标朝向
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal_theta = yaw

        # 重置
        self.geometric_path = []
        self.path_handler.set_global_path([])
        self.last_replan_time = 0
        self.mppi_tracker.reset()

        # 记录指标
        if self.metrics and self.current_state is not None:
            self.metrics.set_goal(self.current_state[:2], self.goal_pos)

        rospy.loginfo(f"[MPPIRidgeback] New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")

    def _scan_cb(self, msg):
        """激光雷达回调"""
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
            rospy.loginfo(f"[MPPIRidgeback] Map: {msg.info.width}x{msg.info.height}")

        self.global_planner.load_static_map(msg)
        self.map_received = True

        # 将 Costmap 传给 MPPI
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
        if dist < 0.25:  # Ridgeback 更大，放宽到达判定
            if self.metrics:
                self.metrics.finish(success=True)
            rospy.loginfo("[MPPIRidgeback] Goal reached!")
            self.goal_pos = None
            self.geometric_path = []
            self.pub_cmd.publish(Twist())
            return

        # 定期重规划
        current_time = rospy.Time.now().to_sec()
        need_replan = (current_time - self.last_replan_time > self.replan_interval or
                       len(self.geometric_path) == 0)

        if need_replan:
            self._do_global_planning()
            self.last_replan_time = current_time

        # 检查路径有效
        if self.path_handler.is_path_finished(self.current_state):
            self.pub_cmd.publish(Twist())
            return

        # PathHandler 更新
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

        # MPPI 求解
        t_start = time.time()
        control, mppi_traj = self.mppi_tracker.solve(
            self.current_state,
            ref_window,
            obstacles_list=self.obstacle_points
        )
        solve_time = time.time() - t_start

        # 一阶低通滤波
        filtered_control = self.filter_alpha * control + (1 - self.filter_alpha) * self.last_cmd
        self.last_cmd = filtered_control

        # 发布控制 (3维: vx, vy, omega)
        cmd = Twist()
        cmd.linear.x = float(filtered_control[0])
        cmd.linear.y = float(filtered_control[1])
        cmd.angular.z = float(filtered_control[2])
        self.pub_cmd.publish(cmd)

        # 记录指标
        if self.metrics:
            self._record_metrics(filtered_control, ref_window, solve_time)

        # 可视化
        self._visualize(mppi_traj)

        rospy.loginfo_throttle(1.0,
            f"[MPPIRidgeback] dist={dist:.2f}m, vx={control[0]:.2f}, vy={control[1]:.2f}, "
            f"omega={control[2]:.2f}, t={solve_time*1000:.1f}ms")

    def _do_global_planning(self):
        """执行全局规划"""
        if self.goal_pos is None:
            return

        # 检查是否使用无地图模式
        current_time = rospy.Time.now().to_sec()
        use_mapless = False

        if not self.map_received:
            if self.mapless_mode:
                # 等待一段时间后启用无地图模式
                if current_time - self.start_time > self.map_wait_time:
                    use_mapless = True
                    rospy.loginfo_once("[MPPIRidgeback] No map available, using mapless mode (direct path + laser avoidance)")
                else:
                    rospy.loginfo_throttle(2.0, f"[MPPIRidgeback] Waiting for map... ({self.map_wait_time - (current_time - self.start_time):.1f}s)")
                    return
            else:
                rospy.logwarn_throttle(5.0, "[MPPIRidgeback] No map received, waiting...")
                return

        if use_mapless:
            # 无地图模式: 生成直线路径
            path = self._generate_direct_path()
        else:
            # 有地图模式: 使用 A* 规划
            self.global_planner.update_dynamic_obstacles(self.obstacle_points)
            path = self.global_planner.plan(self.current_state[:2], self.goal_pos)

        if path is None:
            rospy.logwarn("[MPPIRidgeback] Global planning failed!")
            return

        self.geometric_path = path
        self.path_handler.set_global_path(path, self.current_state, dt=self.dt)

        rospy.loginfo(f"[MPPIRidgeback] Plan: {len(path)} waypoints -> "
                      f"{len(self.path_handler.active_path)} trajectory points")

    def _generate_direct_path(self):
        """
        生成直线路径 (无地图模式)

        在没有静态地图时，生成从当前位置到目标的直线路径。
        避障完全依赖 MPPI 的障碍物代价函数 (使用激光雷达数据)。
        """
        if self.current_state is None or self.goal_pos is None:
            return None

        start = self.current_state[:2]
        goal = self.goal_pos

        # 计算直线距离
        dist = np.linalg.norm(goal - start)
        if dist < 0.1:
            return [tuple(start), tuple(goal)]

        # 生成中间点 (每 0.5m 一个点)
        n_points = max(2, int(dist / 0.5))
        path = []

        for i in range(n_points + 1):
            alpha = i / n_points
            x = start[0] + alpha * (goal[0] - start[0])
            y = start[1] + alpha * (goal[1] - start[1])
            path.append((x, y))

        return path

    def _record_metrics(self, control, ref_window, solve_time=None):
        """记录性能指标"""
        if not self.metrics:
            return

        ref_point = ref_window[0] if ref_window else None

        self.metrics.record(
            pose=self.current_state,
            control=control,
            ref_point=ref_point,
            solve_time=solve_time
        )

    # ==================== 可视化 ====================

    def _visualize(self, mppi_traj):
        """发布可视化消息"""
        now = rospy.Time.now()

        # 全局路径
        if self.geometric_path:
            self._publish_path(self.pub_global_path, self.geometric_path, now)

        # 活跃路径
        active_path = self.path_handler.get_full_active_path()
        if active_path:
            pts = [(p['x'], p['y']) for p in active_path]
            self._publish_path(self.pub_active_path, pts, now)

        # MPPI 预测轨迹
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
        m.scale.x = m.scale.y = 0.4
        m.scale.z = 0.2
        m.color.a = 0.8
        m.color.r = 1.0
        m.color.g = 0.5
        self.pub_goal_marker.publish(m)


if __name__ == '__main__':
    try:
        MPPIRidgebackController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
