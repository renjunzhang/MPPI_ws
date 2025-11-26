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
        
        # --- 3. MPPI 参数配置 ---
        self.v_max = 0.5  # 降低最大速度提高控制精度
        self.w_max = 2.0
        
        noise_sigma = torch.tensor([[0.15, 0.0], [0.0, 1.5]], device=self.device)
        
        self.mppi = MPPI(self.dynamics, self.running_cost, nx=3, 
                         noise_sigma=noise_sigma,
                         num_samples=800,  # 增加采样数
                         horizon=40,       # 适中的预测视野
                         device=self.device,
                         u_min=torch.tensor([0.0, -self.w_max], device=self.device),  # 不允许倒车
                         u_max=torch.tensor([self.v_max, self.w_max], device=self.device))
        
        # --- 4. ROS 接口 ---
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_markers = rospy.Publisher('/mppi_paths', MarkerArray, queue_size=1)
        
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
                cost_goal_heading = torch.abs(angle_diff) * 2.0
                
                cost_goal = cost_goal_dist + cost_goal_heading
            else:
                dist_to_goal = torch.zeros_like(x)
                cost_goal = torch.zeros_like(x)
                
            # --- 2. 障碍物代价 (基于激光雷达) ---
            cost_obs = torch.zeros_like(x)
            safe_margin = 0.25  # 安全距离
            
            # 遍历所有检测到的障碍物点
            for obs_x, obs_y in self.obstacle_points:
                dist_to_obs = torch.sqrt((x - obs_x)**2 + (y - obs_y)**2 + 1e-6)
                # 分段惩罚
                penetration = torch.clamp(safe_margin - dist_to_obs, min=0)
                cost_obs = cost_obs + (penetration ** 2) * 300.0
            
            # --- 3. 控制平滑代价 ---
            cost_ctrl = (v**2 * 0.01 + action[:, 1]**2 * 0.02)

            # --- 4. 终点减速逻辑 ---
            near_goal_mask = (dist_to_goal < 0.5).float()
            cost_stop = near_goal_mask * (v**2) * 10.0 
            
            return cost_goal + cost_obs + cost_ctrl + cost_stop

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
            return

        # 2. 待命检查：如果没有目标，强制停车
        if self.goal_pos is None:
            # 发送0速度，防止滑动
            stop = Twist()
            self.pub_cmd.publish(stop)
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
        
        # 4. 发布命令
        cmd = Twist()
        cmd.linear.x = action[0].item()
        cmd.angular.z = action[1].item()
        self.pub_cmd.publish(cmd)
        
        # 5. 可视化
        self.visualize_trajectories()

    # 辅助函数：停车
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def visualize_trajectories(self):
        U = self.mppi.U
        if U is None: return
        
        state = self.current_state.clone()
        path_points = [Point(x=state[0].item(), y=state[1].item(), z=0.0)]
        
        for t in range(U.shape[0]):
            state = self.dynamics(state.unsqueeze(0), U[t].unsqueeze(0)).squeeze(0)
            path_points.append(Point(x=state[0].item(), y=state[1].item(), z=0.0))

        marker = Marker()
        marker.header.frame_id = "odom" # 使用 odom 坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mppi_pred"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.orientation.w = 1.0 # 修复报错
        marker.points = path_points
        
        ma = MarkerArray()
        ma.markers.append(marker)
        self.pub_markers.publish(ma)

if __name__ == '__main__':
    try:
        MPPIController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass