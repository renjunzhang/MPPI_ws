#!/usr/bin/env python3
"""
MPC + A* 全局路径规划
- A* 负责全局路径搜索，避开障碍物
- MPC 负责局部轨迹跟踪，平滑控制

支持两种地图模式：
1. 静态地图模式：使用 map_server 提供的地图 (/map topic)
2. 动态地图模式：仅使用激光雷达实时构建局部地图

这是正宗的分层规划架构：
1. 全局规划层 (A*): 生成绕过障碍物的路径点序列
2. 局部规划层 (MPC): 跟踪最近的路径点，生成平滑控制
"""

import rospy
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64, Float64MultiArray
from tf.transformations import euler_from_quaternion
import heapq
import time


class AStarPlanner:
    """A* 全局路径规划器 - 支持静态地图和动态障碍物"""
    
    def __init__(self, resolution=0.05, map_size=20.0):
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)
        self.origin_x = -map_size / 2
        self.origin_y = -map_size / 2
        
        # 障碍物膨胀半径
        self.inflate_radius = 0.25
        self.inflate_cells = int(self.inflate_radius / resolution)
        
        # 栅格地图
        self.static_grid = None  # 静态地图 (来自 map_server)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        self.has_static_map = False
        
        rospy.loginfo(f"[A*] Initialized: {self.grid_size}x{self.grid_size}, resolution: {resolution}m")
        
    def load_static_map(self, map_msg):
        """从 OccupancyGrid 消息加载静态地图"""
        # 获取地图参数
        width = map_msg.info.width
        height = map_msg.info.height
        map_resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        rospy.loginfo(f"[A*] Loading static map: {width}x{height}, resolution: {map_resolution}m")
        rospy.loginfo(f"[A*] Map origin: ({origin_x}, {origin_y})")
        
        # 更新规划器参数以匹配地图
        self.resolution = map_resolution
        self.grid_size = max(width, height)
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # 转换地图数据
        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        map_data = np.array(map_msg.data).reshape((height, width))
        
        # 创建二值地图
        self.static_grid = np.zeros((width, height), dtype=np.uint8)
        self.static_grid[map_data.T > 50] = 1  # 占用
        self.static_grid[map_data.T < 0] = 1   # 未知区域也视为障碍
        
        # 膨胀静态障碍物
        self.inflate_cells = int(self.inflate_radius / self.resolution)
        self.static_grid = self.inflate_map(self.static_grid)
        
        self.grid = self.static_grid.copy()
        self.has_static_map = True
        
        occupied = np.sum(self.static_grid)
        rospy.loginfo(f"[A*] Static map loaded! {occupied} occupied cells (after inflation)")
    
    def inflate_map(self, grid):
        """膨胀障碍物"""
        inflated = grid.copy()
        obstacles = np.argwhere(grid == 1)
        
        for ox, oy in obstacles:
            for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                for dy in range(-self.inflate_cells, self.inflate_cells + 1):
                    nx, ny = ox + dx, oy + dy
                    if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                        if dx*dx + dy*dy <= self.inflate_cells * self.inflate_cells:
                            inflated[nx, ny] = 1
        return inflated
        
    def world_to_grid(self, x, y):
        """世界坐标转栅格坐标"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """栅格坐标转世界坐标"""
        x = gx * self.resolution + self.origin_x + self.resolution / 2
        y = gy * self.resolution + self.origin_y + self.resolution / 2
        return x, y
    
    def update_obstacles(self, obstacle_points):
        """更新障碍物地图（结合静态地图和动态障碍物）"""
        # 如果有静态地图，从静态地图开始
        if self.has_static_map and self.static_grid is not None:
            self.grid = self.static_grid.copy()
        else:
            self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # 添加动态障碍物（激光雷达检测的）
        obstacle_count = 0
        for ox, oy in obstacle_points:
            gx, gy = self.world_to_grid(ox, oy)
            
            if not (0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]):
                continue
            
            obstacle_count += 1
            
            # 膨胀障碍物
            for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                for dy in range(-self.inflate_cells, self.inflate_cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid.shape[0] and 0 <= ny < self.grid.shape[1]:
                        if dx*dx + dy*dy <= self.inflate_cells * self.inflate_cells:
                            self.grid[nx, ny] = 1
        
        occupied = np.sum(self.grid)
        map_type = "static+dynamic" if self.has_static_map else "dynamic only"
        rospy.loginfo_throttle(2.0, f"[A*] Map ({map_type}): {obstacle_count} dynamic obs, {occupied} total occupied")
    
    def heuristic(self, a, b):
        """启发式函数：欧几里得距离"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, node):
        """获取邻居节点 (8方向)"""
        neighbors = []
        grid_h, grid_w = self.grid.shape
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = node[0] + dx, node[1] + dy
                if 0 <= nx < grid_h and 0 <= ny < grid_w:
                    if self.grid[nx, ny] == 0:  # 非障碍物
                        # 对角线移动代价更高
                        cost = 1.414 if dx != 0 and dy != 0 else 1.0
                        neighbors.append(((nx, ny), cost))
        return neighbors
    
    def plan(self, start_xy, goal_xy):
        """A* 路径搜索"""
        start = self.world_to_grid(start_xy[0], start_xy[1])
        goal = self.world_to_grid(goal_xy[0], goal_xy[1])
        
        grid_h, grid_w = self.grid.shape
        
        rospy.loginfo(f"[A*] Planning from grid {start} to {goal}, map size: {grid_h}x{grid_w}")
        rospy.loginfo(f"[A*] World coords: ({start_xy[0]:.2f}, {start_xy[1]:.2f}) -> ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
        
        # 检查起点和终点是否有效
        if not (0 <= start[0] < grid_h and 0 <= start[1] < grid_w):
            rospy.logwarn(f"[A*] Start {start} out of map ({grid_h}x{grid_w})!")
            return None
        if not (0 <= goal[0] < grid_h and 0 <= goal[1] < grid_w):
            rospy.logwarn(f"[A*] Goal {goal} out of map ({grid_h}x{grid_w})!")
            return None
        
        # 如果起点在障碍物中，尝试找最近的自由点
        if self.grid[start[0], start[1]] == 1:
            rospy.logwarn("[A*] Start in obstacle, finding nearest free...")
            start = self.find_nearest_free(start)
            if start is None:
                rospy.logwarn("[A*] Start in obstacle, cannot find free cell!")
                return None
            rospy.loginfo(f"[A*] Moved start to {start}")
        
        # 如果终点在障碍物中，尝试找最近的自由点
        if self.grid[goal[0], goal[1]] == 1:
            rospy.logwarn("[A*] Goal in obstacle, finding nearest free...")
            goal = self.find_nearest_free(goal)
            if goal is None:
                rospy.logwarn("[A*] Goal in obstacle, cannot find free cell!")
                return None
            rospy.loginfo(f"[A*] Moved goal to {goal}")
        
        # A* 搜索
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        iterations = 0
        max_iterations = 50000  # 防止无限循环
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # 重建路径
                path = []
                temp = current
                while temp in came_from:
                    wx, wy = self.grid_to_world(temp[0], temp[1])
                    path.append((wx, wy))
                    temp = came_from[temp]
                # 添加起点
                wx, wy = self.grid_to_world(start[0], start[1])
                path.append((wx, wy))
                path.reverse()
                
                rospy.loginfo(f"[A*] Path found! {len(path)} raw points, {iterations} iterations")
                
                # 简化路径（移除共线点）
                path = self.simplify_path(path)
                return path
            
            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        rospy.logwarn(f"[A*] No path found after {iterations} iterations!")
        return None
    
    def find_nearest_free(self, point, max_radius=30):
        """找到最近的自由栅格"""
        grid_h, grid_w = self.grid.shape
        for r in range(1, max_radius):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = point[0] + dx, point[1] + dy
                        if 0 <= nx < grid_h and 0 <= ny < grid_w:
                            if self.grid[nx, ny] == 0:
                                return (nx, ny)
        return None
    
    def simplify_path(self, path, tolerance=0.05):
        """简化路径，移除共线点，但保留关键转折点"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            p0 = simplified[-1]
            p1 = path[i]
            p2 = path[i + 1]
            
            # 计算向量
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            
            # 计算叉积（判断方向变化）
            cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
            
            # 计算向量长度
            len1 = np.sqrt(v1[0]**2 + v1[1]**2)
            len2 = np.sqrt(v2[0]**2 + v2[1]**2)
            
            # 归一化叉积
            if len1 > 0 and len2 > 0:
                normalized_cross = cross / (len1 * len2)
            else:
                normalized_cross = 0
            
            # 如果方向变化显著，保留该点
            if normalized_cross > tolerance:
                simplified.append(p1)
        
        simplified.append(path[-1])
        
        rospy.loginfo(f"[A*] Path simplified: {len(path)} -> {len(simplified)} points")
        return simplified


class MPCWithAStarController:
    def __init__(self):
        rospy.init_node('mpc_astar_controller')
        rospy.loginfo("MPC + A* Controller Initializing...")
        
        # === 1. 状态变量 ===
        self.goal_pos = None
        self.current_state = None
        self.obstacle_points = []
        self.last_control = np.array([0.0, 0.0])
        
        # 全局路径
        self.global_path = []
        self.current_waypoint_idx = 0
        self.waypoint_reach_dist = 0.3  # 到达路径点的距离阈值
        
        # === 2. A* 规划器 ===
        self.astar = AStarPlanner(resolution=0.05, map_size=12.0)  # 更细的分辨率
        self.replan_interval = 1.0  # 更频繁重新规划
        self.last_replan_time = 0
        
        # === 3. MPC 参数 ===
        self.dt = 0.1
        self.N = 15  # 减少预测步数，因为有全局路径引导
        
        self.v_max = 0.22
        self.v_min = -0.05
        self.w_max = 2.0
        
        # 代价权重
        self.Q_dist = 20.0      # 到路径点距离
        self.Q_theta = 3.0      # 朝向
        self.R_v = 0.1
        self.R_w = 0.2
        self.R_dv = 0.5
        self.R_dw = 0.3
        
        # === 4. 构建 MPC ===
        self.setup_mpc()
        
        # === 5. ROS 接口 ===
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_local_path = rospy.Publisher('/mpc_predict_path', Path, queue_size=1)
        self.pub_markers = rospy.Publisher('/mpc_markers', MarkerArray, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.pub_waypoint_marker = rospy.Publisher('/waypoint_marker', Marker, queue_size=1)
        self.pub_obstacle_markers = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=1)
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)  # 订阅静态地图
        
        # 等待地图加载
        self.map_received = False
        rospy.loginfo("[MPC+A*] Waiting for static map from /map topic...")
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        self.predicted_trajectory = []
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("[MPC+A*] Hierarchical Planner Ready")
        rospy.loginfo("  Global Planner: A* (with static map support)")
        rospy.loginfo("  Local Planner: MPC (horizon: 15 steps)")
        rospy.loginfo("=" * 50)

    def setup_mpc(self):
        """构建简化的 MPC (跟踪路径点)"""
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        state = ca.vertcat(x, y, theta)
        n_state = 3
        
        v = ca.SX.sym('v')
        w = ca.SX.sym('w')
        control = ca.vertcat(v, w)
        n_control = 2
        
        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), w)
        self.f = ca.Function('f', [state, control], [rhs])
        
        X = ca.SX.sym('X', n_state, self.N + 1)
        U = ca.SX.sym('U', n_control, self.N)
        
        # 参数: 初始状态 + 目标路径点 + 上一次控制
        P = ca.SX.sym('P', n_state + 2 + n_control)
        
        obj = 0
        g = []
        
        g.append(X[:, 0] - P[:n_state])
        
        x_target = P[n_state]
        y_target = P[n_state + 1]
        v_last = P[n_state + 2]
        w_last = P[n_state + 3]
        
        for k in range(self.N):
            x_k, y_k, theta_k = X[0, k], X[1, k], X[2, k]
            
            # 到目标点距离
            dist_sq = (x_k - x_target)**2 + (y_k - y_target)**2
            obj += self.Q_dist * dist_sq
            
            # 朝向目标
            dx = x_target - x_k
            dy = y_target - y_k
            desired_theta = ca.atan2(dy, dx)
            theta_err = ca.atan2(ca.sin(theta_k - desired_theta), 
                                 ca.cos(theta_k - desired_theta))
            obj += self.Q_theta * theta_err**2
            
            # 控制代价
            obj += self.R_v * U[0, k]**2
            obj += self.R_w * U[1, k]**2
            
            # 控制变化率
            if k == 0:
                dv, dw = U[0, k] - v_last, U[1, k] - w_last
            else:
                dv, dw = U[0, k] - U[0, k-1], U[1, k] - U[1, k-1]
            obj += self.R_dv * dv**2 + self.R_dw * dw**2
            
            # 动力学
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])
            g.append(X[:, k+1] - x_next)
        
        # 终端代价
        terminal_dist = (X[0, self.N] - x_target)**2 + (X[1, self.N] - y_target)**2
        obj += self.Q_dist * 3.0 * terminal_dist
        
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        g = ca.vertcat(*g)
        
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': g, 'p': P}
        
        opts = {
            'ipopt': {
                'max_iter': 50,
                'print_level': 0,
                'acceptable_tol': 1e-4,
                'warm_start_init_point': 'yes'
            },
            'print_time': 0
        }
        
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        self.n_state = n_state
        self.n_control = n_control
        
        self.lbx = []
        self.ubx = []
        for _ in range(self.N + 1):
            self.lbx += [-ca.inf, -ca.inf, -ca.inf]
            self.ubx += [ca.inf, ca.inf, ca.inf]
        for _ in range(self.N):
            self.lbx += [self.v_min, -self.w_max]
            self.ubx += [self.v_max, self.w_max]
        
        n_constraints = n_state * (self.N + 1)
        self.lbg = [0] * n_constraints
        self.ubg = [0] * n_constraints
        
        self.u0 = np.zeros((self.N, self.n_control))
        self.x0 = np.zeros((self.N + 1, self.n_state))

    def plan_global_path(self):
        """使用 A* 规划全局路径"""
        if self.current_state is None or self.goal_pos is None:
            return False
        
        # 更新障碍物地图
        self.astar.update_obstacles(self.obstacle_points)
        
        # 规划路径
        start = self.current_state[:2]
        goal = self.goal_pos
        
        path = self.astar.plan(start, goal)
        
        if path is not None and len(path) > 0:
            self.global_path = path
            self.current_waypoint_idx = 0
            rospy.loginfo(f"[A*] Path found with {len(path)} waypoints")
            return True
        else:
            rospy.logwarn("[A*] Failed to find path!")
            return False

    def get_current_waypoint(self):
        """获取当前应该追踪的路径点"""
        if len(self.global_path) == 0:
            return self.goal_pos
        
        # 检查是否到达当前路径点
        while self.current_waypoint_idx < len(self.global_path):
            wp = self.global_path[self.current_waypoint_idx]
            dist = np.sqrt((self.current_state[0] - wp[0])**2 + 
                          (self.current_state[1] - wp[1])**2)
            
            if dist < self.waypoint_reach_dist:
                self.current_waypoint_idx += 1
                rospy.loginfo(f"[MPC] Reached waypoint {self.current_waypoint_idx}/{len(self.global_path)}")
            else:
                break
        
        # 返回当前路径点（或最后一个）
        if self.current_waypoint_idx < len(self.global_path):
            return np.array(self.global_path[self.current_waypoint_idx])
        else:
            return self.goal_pos

    def solve_mpc(self, current_state, target_point):
        """求解 MPC"""
        p = np.concatenate([
            current_state,
            target_point,
            self.last_control
        ])
        
        x0_guess = np.concatenate([self.x0.flatten(), self.u0.flatten()])
        
        try:
            sol = self.solver(
                x0=x0_guess,
                lbx=self.lbx, ubx=self.ubx,
                lbg=self.lbg, ubg=self.ubg,
                p=p
            )
            
            opt = sol['x'].full().flatten()
            n_state_vars = self.n_state * (self.N + 1)
            x_opt = opt[:n_state_vars].reshape(self.N + 1, self.n_state)
            u_opt = opt[n_state_vars:].reshape(self.N, self.n_control)
            
            # Warm start
            self.x0[:-1, :] = x_opt[1:, :]
            self.x0[-1, :] = x_opt[-1, :]
            self.u0[:-1, :] = u_opt[1:, :]
            self.u0[-1, :] = u_opt[-1, :]
            
            self.predicted_trajectory = x_opt
            
            return u_opt[0], True
            
        except Exception as e:
            rospy.logwarn(f"MPC solve failed: {e}")
            return np.array([0.0, 0.0]), False

    # === 回调函数 ===
    
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state = np.array([x, y, yaw])

    def goal_cb(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.global_path = []
        self.current_waypoint_idx = 0
        self.last_replan_time = 0  # 强制重新规划
        
        # 重置 MPC warm start
        self.u0 = np.zeros((self.N, self.n_control))
        self.x0 = np.zeros((self.N + 1, self.n_state))
        
        rospy.loginfo(f"[MPC+A*] New Goal: [{self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f}]")

    def scan_cb(self, msg):
        if self.current_state is None:
            return
        
        x, y, theta = self.current_state
        obstacle_points = []
        angle = msg.angle_min
        
        # 使用更多的激光点（每2个点采样一次）
        for i, r in enumerate(msg.ranges):
            if i % 2 == 0 and msg.range_min < r < msg.range_max and r < 4.0:
                obs_x = x + r * np.cos(theta + angle)
                obs_y = y + r * np.sin(theta + angle)
                obstacle_points.append((obs_x, obs_y))
            angle += msg.angle_increment
        
        self.obstacle_points = obstacle_points
    
    def map_cb(self, msg):
        """接收静态地图"""
        if not self.map_received:
            rospy.loginfo(f"[MPC+A*] Static map received!")
            rospy.loginfo(f"  - Size: {msg.info.width} x {msg.info.height}")
            rospy.loginfo(f"  - Resolution: {msg.info.resolution}m")
            rospy.loginfo(f"  - Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})")
        
        # 加载静态地图到A*规划器
        self.astar.load_static_map(msg)
        self.map_received = True

    def control_loop(self, event):
        if self.current_state is None:
            rospy.logwarn_throttle(2.0, "[MPC+A*] Waiting for odom...")
            return

        if self.goal_pos is None:
            self.pub_cmd.publish(Twist())
            rospy.loginfo_throttle(5.0, "[MPC+A*] Waiting for goal...")
            return
        
        # 到达检测
        dist_to_goal = np.linalg.norm(self.goal_pos - self.current_state[:2])
        rospy.loginfo_throttle(1.0, f"[MPC+A*] Distance to goal: {dist_to_goal:.2f}m")
        
        if dist_to_goal < 0.15:
            rospy.loginfo("[MPC+A*] Goal Reached!")
            self.goal_pos = None
            self.global_path = []
            self.pub_cmd.publish(Twist())
            return
        
        # 定期重新规划全局路径
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_replan_time > self.replan_interval or len(self.global_path) == 0:
            if self.plan_global_path():
                self.last_replan_time = current_time
        
        # 获取当前目标路径点
        target = self.get_current_waypoint()
        
        # MPC 跟踪
        control, success = self.solve_mpc(self.current_state, target)
        
        if success:
            cmd = Twist()
            cmd.linear.x = float(control[0])
            cmd.angular.z = float(control[1])
            self.pub_cmd.publish(cmd)
            self.last_control = control
        else:
            self.pub_cmd.publish(Twist())
        
        # 可视化
        self.visualize()

    def visualize(self):
        """可视化全局路径、局部轨迹、目标点"""
        # 全局路径
        if len(self.global_path) > 0:
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = rospy.Time.now()
            for wp in self.global_path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.pub_global_path.publish(path_msg)
        
        # 局部轨迹
        if len(self.predicted_trajectory) > 0:
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = rospy.Time.now()
            for state in self.predicted_trajectory:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = state[0]
                pose.pose.position.y = state[1]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.pub_local_path.publish(path_msg)
        
        # 最终目标
        if self.goal_pos is not None:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "goal"
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.goal_pos[0]
            marker.pose.position.y = self.goal_pos[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.2
            marker.color.a = 0.8
            marker.color.g = 1.0
            self.pub_goal_marker.publish(marker)
        
        # 当前追踪的路径点
        if len(self.global_path) > 0 and self.current_waypoint_idx < len(self.global_path):
            wp = self.global_path[self.current_waypoint_idx]
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoint"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            self.pub_waypoint_marker.publish(marker)
        
        # 障碍物
        ma = MarkerArray()
        for i, (ox, oy) in enumerate(self.obstacle_points[::5]):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = rospy.Time.now()
            m.ns = "obs"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.a = 0.6
            m.color.r = 1.0
            m.lifetime = rospy.Duration(0.2)
            ma.markers.append(m)
        self.pub_obstacle_markers.publish(ma)


if __name__ == '__main__':
    try:
        MPCWithAStarController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
