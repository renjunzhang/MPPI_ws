#!/usr/bin/env python3
import rospy
import torch
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
from pytorch_mppi import MPPI

class MPPIController:
    def __init__(self):
        rospy.init_node('mppi_controller')
        
        # --- 1. 硬件配置 ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"MPPI running on: {self.device}")
        
        # --- 2. 状态变量 ---
        self.goal_pos = None  # 初始没有目标
        self.current_state = None # [x, y, yaw]
        self.obstacle_points = []  # 存储激光雷达检测到的障碍物点
        self.last_action = torch.zeros(2, device=torch.device("cuda" if torch.cuda.is_available() else "cpu"))  # 上一次的控制量，用于平滑
        
        # --- 3. MPPI 参数配置 ---
        self.v_max = 0.5  # 降低最大速度防止过冲
        self.w_max = 0.8  # 降低最大角速度使转向更平滑
        
        # 减小噪声使轨迹更平滑
        noise_sigma = torch.tensor([[0.08, 0.0], [0.0, 0.6]], device=self.device)
        
        self.mppi = MPPI(self.dynamics, self.running_cost, nx=3, 
                         noise_sigma=noise_sigma,
                         num_samples=1000,  # 增加采样数提高规划质量
                         horizon=50,        # 增加预测视野
                         lambda_=1.0,       # 温度参数，越大越平滑
                         device=self.device,
                         u_min=torch.tensor([0.0, -self.w_max], device=self.device),
                         u_max=torch.tensor([self.v_max, self.w_max], device=self.device))
        
        # --- 4. ROS 接口 ---
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_markers = rospy.Publisher('/mppi_paths', MarkerArray, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.pub_obstacle_markers = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=1)
        self.pub_robot_marker = rospy.Publisher('/robot_marker', Marker, queue_size=1)
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)  # 订阅激光雷达
        
        # 定时器 20Hz
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        rospy.loginfo("MPPI Controller Ready. Waiting for Goal via RViz...")

    # ==========================================
    # 核心算法部分 (现在都在类里面了)
    # ==========================================
    
    def dynamics(self, state, action):
        """ 差速机器人动力学模型 """
        x = state[:, 0]
        y = state[:, 1]
        theta = state[:, 2]
        
        v = action[:, 0]
        w = action[:, 1]
        dt = 0.05
        
        new_x = x + v * torch.cos(theta) * dt
        new_y = y + v * torch.sin(theta) * dt
        new_theta = theta + w * dt
        
        return torch.stack((new_x, new_y, new_theta), dim=1)

    def running_cost(self, state, action):
            """ 代价函数 """
            x = state[:, 0]
            y = state[:, 1]
            theta = state[:, 2]
            v = action[:, 0] # 获取线速度
            
            # --- 1. 目标代价 ---
            if self.goal_pos is not None:
                dx = self.goal_pos[0] - x  # 目标 - 当前
                dy = self.goal_pos[1] - y
                dist_to_goal = torch.sqrt(dx**2 + dy**2 + 1e-6)
                
                # 距离代价 - 增大权重
                cost_goal_dist = dist_to_goal * 5.0
                
                # 朝向代价 - 鼓励朝向目标
                desired_angle = torch.atan2(dy, dx)
                angle_diff = torch.remainder(desired_angle - theta + np.pi, 2*np.pi) - np.pi
                cost_goal_heading = torch.abs(angle_diff) * 3.0
                
                cost_goal = cost_goal_dist + cost_goal_heading
            else:
                dist_to_goal = torch.zeros_like(x)
                cost_goal = torch.zeros_like(x)
                
            # --- 2. 障碍物代价 (基于激光雷达) ---
            cost_obs = torch.zeros_like(x)
            safe_margin = 0.35  # 安全距离
            
            # 遍历所有检测到的障碍物点
            for obs_x, obs_y in self.obstacle_points:
                dist_to_obs = torch.sqrt((x - obs_x)**2 + (y - obs_y)**2 + 1e-6)
                # 分段惩罚
                penetration = torch.clamp(safe_margin - dist_to_obs, min=0)
                cost_obs = cost_obs + (penetration ** 2) * 300.0
            
            # --- 3. 控制平滑代价 ---
            # 增大角速度惩罚使转向更平滑
            cost_ctrl = (v**2 * 0.02 + action[:, 1]**2 * 0.1)
            
            # --- 4. 角速度变化率惩罚 (使轨迹更平滑) ---
            # 惩罚角速度的大幅变化
            w_rate_penalty = (action[:, 1] - self.last_action[1].item())**2 * 0.3

            # --- 5. 终点减速逻辑 (优化：增大减速区域防止过冲) ---
            # 扩大减速区域，增大减速权重
            near_goal_mask = (dist_to_goal < 0.5).float()
            cost_stop = near_goal_mask * (v**2) * 15.0
            
            # 极近目标时更强的减速
            very_near_mask = (dist_to_goal < 0.2).float()
            cost_stop = cost_stop + very_near_mask * (v**2) * 30.0 
            
            return cost_goal + cost_obs + cost_ctrl + w_rate_penalty + cost_stop

    # ==========================================
    # 回调函数
    # ==========================================
    
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state = torch.tensor([x, y, yaw], device=self.device)

    def goal_cb(self, msg):
        # 收到新目标时，更新目标点
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.goal_pos = torch.tensor([gx, gy], device=self.device)
        rospy.loginfo(f"New Goal Received: [{gx:.2f}, {gy:.2f}]")

    def scan_cb(self, msg):
        """ 处理激光雷达数据，转换为世界坐标系中的障碍物点 """
        if self.current_state is None:
            return
        
        # 获取当前位姿
        x = self.current_state[0].item()
        y = self.current_state[1].item()
        theta = self.current_state[2].item()
        
        obstacle_points = []
        angle = msg.angle_min
        
        # 只取部分点降低计算量（每隔10个点取一个）
        for i, r in enumerate(msg.ranges):
            if i % 10 == 0 and msg.range_min < r < msg.range_max:
                # 转换到世界坐标系
                obs_x = x + r * np.cos(theta + angle)
                obs_y = y + r * np.sin(theta + angle)
                
                # 只保留2米内的障碍物点
                if r < 2.0:
                    obstacle_points.append((obs_x, obs_y))
            
            angle += msg.angle_increment
        
        self.obstacle_points = obstacle_points

    def control_loop(self, event):
        # 1. 安全检查
        if self.current_state is None:
            rospy.logwarn_throttle(2.0, "Waiting for odom data...")
            return

        # 2. 待命检查：如果没有目标，强制停车
        if self.goal_pos is None:
            # 发送0速度，防止滑动
            stop = Twist()
            self.pub_cmd.publish(stop)
            rospy.loginfo_throttle(5.0, "No goal set. Waiting for goal...")
            return
        
        # ==========================================
        # 3. 新增：到达检测逻辑 (Arrival Check)
        # ==========================================
        # 计算当前位置到目标的距离
        dx = self.goal_pos[0] - self.current_state[0]
        dy = self.goal_pos[1] - self.current_state[1]
        dist_to_goal = torch.sqrt(dx**2 + dy**2).item() # 转成 python float

        # 设定容差阈值 (例如 0.15米)
        if dist_to_goal < 0.15:
            rospy.loginfo("Goal Reached! Stopping.")
            self.goal_pos = None  # 清除目标，进入待命状态
            self.stop_robot()     # 强制停车
            return
        # ==========================================

        # 3. 计算 MPPI
        action = self.mppi.command(self.current_state)
        
        # 保存当前动作用于下一次平滑计算
        self.last_action = action.clone()
        
        # 4. 发布命令
        cmd = Twist()
        cmd.linear.x = action[0].item()
        cmd.angular.z = action[1].item()
        self.pub_cmd.publish(cmd)
        
        # 5. 可视化
        self.visualize_trajectories()
        self.visualize_goal()
        self.visualize_obstacles()
        self.visualize_robot()

    # 辅助函数：停车
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def visualize_trajectories(self):
        U = self.mppi.U
        if U is None: return
        
        ma = MarkerArray()
        
        # --- 最优轨迹 (红色粗线) ---
        state = self.current_state.clone()
        path_points = [Point(x=state[0].item(), y=state[1].item(), z=0.0)]
        
        for t in range(U.shape[0]):
            state = self.dynamics(state.unsqueeze(0), U[t].unsqueeze(0)).squeeze(0)
            path_points.append(Point(x=state[0].item(), y=state[1].item(), z=0.0))

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mppi_optimal"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # 加粗线条
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.points = path_points
        ma.markers.append(marker)
        
        # --- 轨迹点标记 (黄色小球) ---
        for i, pt in enumerate(path_points[::5]):  # 每5个点显示一个
            pt_marker = Marker()
            pt_marker.header.frame_id = "odom"
            pt_marker.header.stamp = rospy.Time.now()
            pt_marker.ns = "mppi_points"
            pt_marker.id = i + 100
            pt_marker.type = Marker.SPHERE
            pt_marker.action = Marker.ADD
            pt_marker.pose.position = pt
            pt_marker.pose.orientation.w = 1.0
            pt_marker.scale.x = 0.06
            pt_marker.scale.y = 0.06
            pt_marker.scale.z = 0.06
            pt_marker.color.a = 0.8
            pt_marker.color.r = 1.0
            pt_marker.color.g = 1.0
            pt_marker.color.b = 0.0
            ma.markers.append(pt_marker)
        
        self.pub_markers.publish(ma)
    
    def visualize_goal(self):
        """ 可视化目标点 """
        if self.goal_pos is None:
            return
            
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_pos[0].item()
        marker.pose.position.y = self.goal_pos[1].item()
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.2
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.pub_goal_marker.publish(marker)
    
    def visualize_obstacles(self):
        """ 可视化检测到的障碍物点 """
        ma = MarkerArray()
        
        for i, (obs_x, obs_y) in enumerate(self.obstacle_points):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obs_x
            marker.pose.position.y = obs_y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.6
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0.2)  # 短暂显示
            ma.markers.append(marker)
        
        self.pub_obstacle_markers.publish(ma)
    
    def visualize_robot(self):
        """ 可视化机器人当前位置和朝向 """
        if self.current_state is None:
            return
            
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 使用 pose 方式而不是 points 方式
        theta = self.current_state[2].item()
        marker.pose.position.x = self.current_state[0].item()
        marker.pose.position.y = self.current_state[1].item()
        marker.pose.position.z = 0.15
        
        # 设置方向 (yaw 角度转四元数)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(theta / 2.0)
        marker.pose.orientation.w = np.cos(theta / 2.0)
        
        # 使用 pose 方式时，scale 表示箭头尺寸
        marker.scale.x = 0.3   # 箭头长度
        marker.scale.y = 0.08  # 箭头宽度
        marker.scale.z = 0.08  # 箭头高度
        
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        self.pub_robot_marker.publish(marker)

if __name__ == '__main__':
    try:
        MPPIController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass