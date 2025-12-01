#!/usr/bin/env python3
"""
三层规划控制架构 - 主控制器
============================

架构:
    Layer 1: 全局路径规划 (A*) - global_planner.py
    Layer 2: 轨迹生成 (时间参数化) - local_planner.py  
    Layer 3: 轨迹跟踪 (MPC) - mpc_tracker.py

本文件整合三层模块，提供 ROS 接口。

作者: GitHub Copilot
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 导入三层模块
from global_planner import GlobalPathPlanner
from local_planner import TrajectoryGenerator
from mpc_tracker import MPCTracker
from mppi_node import MPPITracker
from ilqr_tracker import ILQRTracker

# ========== 算法选择 (修改此处切换) ==========
# 可选: 'MPC', 'MPPI', 'iLQR'
TRACKER_TYPE = 'MPPI'
# =============================================


class ThreeLayerController:
    """
    三层规划控制器
    
    整合全局规划、轨迹生成、MPC跟踪三个层次，
    实现从目标点到控制指令的完整流程。
    """
    
    def __init__(self):
        rospy.init_node('three_layer_controller')
        
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

        # ===== 时间与预测步长参数 =====
        self.dt = 0.1          # 控制周期
        self.N = 20            # iLQR 预测步数
        self.replan_interval = 3.0
        self.last_replan_time = 0.0
        
        # 三层模块
        self.global_planner = GlobalPathPlanner(resolution=0.05, inflate_radius=0.25)
        self.traj_generator = TrajectoryGenerator(v_max=0.18, v_min=0.08, w_max=1.2)
        self.tracker_type = TRACKER_TYPE
        self.mpc_tracker = self._create_tracker(self.tracker_type)

        
        # 规划结果
        self.geometric_path = []
        self.reference_trajectory = None
        self.traj_start_time = None
        
        # 重规划参数
        self.replan_interval = 3.0
        self.last_replan_time = 0
        # self.dt = 0.1
        
        # ROS 接口
        self._setup_ros()
        
        rospy.loginfo("[Controller] Ready")
    
    def _print_banner(self):
        """打印启动信息"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("Three-Layer Hierarchical Controller")
        rospy.loginfo("  L1: Global Path Planning (A*)")
        rospy.loginfo("  L2: Trajectory Generation")
        rospy.loginfo(f"  L3: Trajectory Tracking ({TRACKER_TYPE})")
        rospy.loginfo("=" * 50)

    def _create_tracker(self, tracker_type):
        """根据类型创建跟踪器"""
        if tracker_type == 'MPC':
            rospy.loginfo("[Controller] Using MPC tracker")
            return MPCTracker(dt=self.dt, N=self.N)
        elif tracker_type == 'MPPI':
            rospy.loginfo("[Controller] Using MPPI tracker")
            return MPPITracker(dt=self.dt, N=30)
        elif tracker_type == 'iLQR':
            rospy.loginfo("[Controller] Using iLQR tracker")
            return ILQRTracker(dt=self.dt, N=self.N, v_max=0.22, v_min=-0.05, w_max=2.0)
        else:
            rospy.logwarn(f"[Controller] Unknown tracker type: {tracker_type}, fallback to MPC")
            return MPCTracker(dt=self.dt, N=self.N)
    
    def _setup_ros(self):
        """初始化 ROS 接口"""
        # Publishers
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_ref_traj = rospy.Publisher('/reference_trajectory', Path, queue_size=1)
        self.pub_mpc_traj = rospy.Publisher('/mpc_predict_path', Path, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.pub_ref_marker = rospy.Publisher('/current_ref_marker', Marker, queue_size=1)
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
        self.reference_trajectory = None
        self.traj_start_time = None
        self.last_replan_time = 0
        self.mpc_tracker.reset()
        self._metrics_poses = []
        self._metrics_speeds = []
        self._metrics_ctes = []
        rospy.loginfo(f"[Controller] New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")
    
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
            rospy.loginfo(f"[Controller] Map: {msg.info.width}x{msg.info.height}")
        self.global_planner.load_static_map(msg)
        self.map_received = True
    
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
            rospy.loginfo("[Controller] Goal reached!")
            self.goal_pos = None
            self.reference_trajectory = None
            self.pub_cmd.publish(Twist())
            return
        
        # 定期重规划
        current_time = rospy.Time.now().to_sec()
        need_replan = (current_time - self.last_replan_time > self.replan_interval or
                       self.reference_trajectory is None)
        
        if need_replan:
            self._do_planning()
            self.last_replan_time = current_time
        
        if self.reference_trajectory is None:
            self.pub_cmd.publish(Twist())
            return
        
        # MPC 跟踪
        elapsed = current_time - self.traj_start_time
        ref_window = self.traj_generator.get_reference_window(
            self.reference_trajectory, elapsed, self.mpc_tracker.N, self.dt)
        
        if not ref_window or len(ref_window) < self.mpc_tracker.N + 1:
            self._do_planning()
            self.last_replan_time = current_time
            return
        
        control, mpc_traj = self.mpc_tracker.solve(self.current_state, ref_window)
        # 修改后的调用 (传入 self.obstacle_points)：
        # control, mpc_traj = self.mpc_tracker.solve(
        #     self.current_state, 
        #     ref_window, 
        #     self.obstacle_points  # <--- 关键！把雷达数据传进去
        # )
        
        # 发布控制
        cmd = Twist()
        cmd.linear.x = float(control[0])
        cmd.angular.z = float(control[1])
        self.pub_cmd.publish(cmd)

        # 记录性能指标
        self._metrics_poses.append((self.current_state[0], self.current_state[1]))
        self._metrics_speeds.append(float(control[0]))
        if ref_window:
            cte = np.sqrt((self.current_state[0]-ref_window[0]['x'])**2 + (self.current_state[1]-ref_window[0]['y'])**2)
            # 过滤异常值（初始帧可能参考点异常）
            if cte < 0.5:
                self._metrics_ctes.append(cte)
        
        # 可视化
        self._visualize(ref_window, mpc_traj)
        
        rospy.loginfo_throttle(1.0, f"[Controller] dist={dist:.2f}m, v={control[0]:.2f}, w={control[1]:.2f}")
    
    def _do_planning(self):
        """执行规划 (Layer 1 + Layer 2)"""
        if not self.map_received or self.goal_pos is None:
            return
        
        # Layer 1: 全局规划
        self.global_planner.update_dynamic_obstacles(self.obstacle_points)
        path = self.global_planner.plan(self.current_state[:2], self.goal_pos)
        
        if path is None:
            rospy.logwarn("[Controller] Global planning failed!")
            return
        
        self.geometric_path = path
        
        # Layer 2: 轨迹生成
        self.reference_trajectory = self.traj_generator.generate(
            path, self.current_state, dt=self.dt)
        
        if self.reference_trajectory is None:
            rospy.logwarn("[Controller] Trajectory generation failed!")
            return
        
        self.traj_start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"[Controller] Plan: path={len(path)}, traj={len(self.reference_trajectory)}")
    
    # ==================== 可视化 ====================
    
    def _visualize(self, ref_window, mpc_traj):
        """发布可视化消息"""
        now = rospy.Time.now()
        
        # 全局路径 (绿色)
        if self.geometric_path:
            self._publish_path(self.pub_global_path, self.geometric_path, now)
        
        # 参考轨迹 (蓝色)
        if self.reference_trajectory:
            pts = [(p['x'], p['y']) for p in self.reference_trajectory[::3]]
            self._publish_path(self.pub_ref_traj, pts, now)
        
        # MPC 预测 (红色)
        if mpc_traj is not None:
            pts = [(s[0], s[1]) for s in mpc_traj]
            self._publish_path(self.pub_mpc_traj, pts, now)
        
        # 目标点
        if self.goal_pos is not None:
            self._publish_goal_marker(now)
        
        # 当前参考点
        if ref_window:
            self._publish_ref_marker(ref_window[0], now)
    
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
        m.color.g = 1.0
        self.pub_goal_marker.publish(m)
    
    def _publish_ref_marker(self, ref, stamp):
        """发布当前参考点标记"""
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = stamp
        m.ns = "ref"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = ref['x']
        m.pose.position.y = ref['y']
        m.pose.position.z = 0.1
        q = quaternion_from_euler(0, 0, ref['theta'])
        m.pose.orientation.x = q[0]
        m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]
        m.scale.x = 0.3
        m.scale.y = m.scale.z = 0.08
        m.color.a = 1.0
        m.color.r = m.color.g = 1.0
        self.pub_ref_marker.publish(m)

    def _publish_metrics(self):
        """导航结束时计算并发布性能指标"""
        if len(self._metrics_poses) < 2:
            return
        # 路径长度
        length = sum(np.sqrt((self._metrics_poses[i+1][0]-self._metrics_poses[i][0])**2 +
                             (self._metrics_poses[i+1][1]-self._metrics_poses[i][1])**2)
                     for i in range(len(self._metrics_poses)-1))
        # 平均速度
        avg_speed = np.mean(self._metrics_speeds) if self._metrics_speeds else 0.0
        # 平均/最大横向误差
        avg_cte = np.mean(self._metrics_ctes) if self._metrics_ctes else 0.0
        max_cte = np.max(self._metrics_ctes) if self._metrics_ctes else 0.0
        # 发布 [path_length, avg_speed, avg_cte, max_cte]
        msg = Float32MultiArray()
        msg.data = [float(length), float(avg_speed), float(avg_cte), float(max_cte)]
        self.pub_metrics.publish(msg)
        rospy.loginfo(f"[Metrics] 路径长度:{length:.2f}m, 平均速度:{avg_speed:.2f}m/s, 平均CTE:{avg_cte:.3f}m, 最大CTE:{max_cte:.3f}m")


if __name__ == '__main__':
    try:
        ThreeLayerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
