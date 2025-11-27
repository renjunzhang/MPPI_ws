#!/usr/bin/env python3
"""
完整三层规划架构：
  Layer 1: 全局路径规划 (A*) - 生成几何路径
  Layer 2: 局部轨迹规划 (Trajectory Generator) - 生成时间参数轨迹
  Layer 3: 轨迹跟踪控制 (MPC Tracker) - 跟踪参考轨迹

作者: GitHub Copilot
"""

import rospy
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import heapq


# ============================================================
# Layer 1: 全局路径规划器 (A*)
# ============================================================
class GlobalPathPlanner:
    """A* 全局路径规划 - 输出几何路径"""
    
    def __init__(self, resolution=0.05):
        self.resolution = resolution
        self.inflate_radius = 0.25
        self.inflate_cells = int(self.inflate_radius / resolution)
        
        self.static_grid = None
        self.grid = None
        self.origin_x = 0
        self.origin_y = 0
        self.has_static_map = False
        
        rospy.loginfo("[Layer1-A*] Global Path Planner initialized")
        
    def load_static_map(self, map_msg):
        """加载静态地图"""
        width = map_msg.info.width
        height = map_msg.info.height
        self.resolution = map_msg.info.resolution
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y
        
        map_data = np.array(map_msg.data).reshape((height, width))
        self.static_grid = np.zeros((width, height), dtype=np.uint8)
        self.static_grid[map_data.T > 50] = 1
        self.static_grid[map_data.T < 0] = 1
        
        # 膨胀
        self.inflate_cells = int(self.inflate_radius / self.resolution)
        self.static_grid = self._inflate_map(self.static_grid)
        self.grid = self.static_grid.copy()
        self.has_static_map = True
        
        rospy.loginfo(f"[Layer1-A*] Map loaded: {width}x{height}, resolution: {self.resolution}m")
    
    def _inflate_map(self, grid):
        """膨胀障碍物"""
        from scipy import ndimage
        kernel_size = 2 * self.inflate_cells + 1
        kernel = np.zeros((kernel_size, kernel_size))
        for i in range(kernel_size):
            for j in range(kernel_size):
                if (i - self.inflate_cells)**2 + (j - self.inflate_cells)**2 <= self.inflate_cells**2:
                    kernel[i, j] = 1
        inflated = ndimage.binary_dilation(grid, structure=kernel).astype(np.uint8)
        return inflated
    
    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin_x + self.resolution / 2
        y = gy * self.resolution + self.origin_y + self.resolution / 2
        return x, y
    
    def update_dynamic_obstacles(self, obstacle_points):
        """更新动态障碍物"""
        if self.has_static_map:
            self.grid = self.static_grid.copy()
        else:
            return
            
        for ox, oy in obstacle_points:
            gx, gy = self.world_to_grid(ox, oy)
            if 0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]:
                for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                    for dy in range(-self.inflate_cells, self.inflate_cells + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.grid.shape[0] and 0 <= ny < self.grid.shape[1]:
                            if dx*dx + dy*dy <= self.inflate_cells**2:
                                self.grid[nx, ny] = 1
    
    def plan(self, start_xy, goal_xy):
        """A* 规划，返回几何路径 [(x,y), ...]"""
        if self.grid is None:
            rospy.logwarn("[Layer1-A*] No map available!")
            return None
            
        start = self.world_to_grid(start_xy[0], start_xy[1])
        goal = self.world_to_grid(goal_xy[0], goal_xy[1])
        
        grid_h, grid_w = self.grid.shape
        
        # 边界检查
        if not (0 <= start[0] < grid_h and 0 <= start[1] < grid_w):
            return None
        if not (0 <= goal[0] < grid_h and 0 <= goal[1] < grid_w):
            return None
        
        # 处理起点/终点在障碍物中的情况
        if self.grid[start[0], start[1]] == 1:
            start = self._find_nearest_free(start)
            if start is None:
                return None
        if self.grid[goal[0], goal[1]] == 1:
            goal = self._find_nearest_free(goal)
            if goal is None:
                return None
        
        # A* 搜索
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        iterations = 0
        while open_set and iterations < 50000:
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
                wx, wy = self.grid_to_world(start[0], start[1])
                path.append((wx, wy))
                path.reverse()
                
                rospy.loginfo(f"[Layer1-A*] Path found: {len(path)} points")
                return self._simplify_path(path)
            
            for neighbor, cost in self._get_neighbors(current):
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = np.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
        
        rospy.logwarn("[Layer1-A*] No path found!")
        return None
    
    def _get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = node[0] + dx, node[1] + dy
                if 0 <= nx < self.grid.shape[0] and 0 <= ny < self.grid.shape[1]:
                    if self.grid[nx, ny] == 0:
                        cost = 1.414 if dx != 0 and dy != 0 else 1.0
                        neighbors.append(((nx, ny), cost))
        return neighbors
    
    def _find_nearest_free(self, point, max_radius=30):
        for r in range(1, max_radius):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = point[0] + dx, point[1] + dy
                        if 0 <= nx < self.grid.shape[0] and 0 <= ny < self.grid.shape[1]:
                            if self.grid[nx, ny] == 0:
                                return (nx, ny)
        return None
    
    def _simplify_path(self, path, tolerance=0.08):
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            p0, p1, p2 = simplified[-1], path[i], path[i + 1]
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
            len1 = np.sqrt(v1[0]**2 + v1[1]**2)
            len2 = np.sqrt(v2[0]**2 + v2[1]**2)
            if len1 > 0 and len2 > 0:
                if cross / (len1 * len2) > tolerance:
                    simplified.append(p1)
        simplified.append(path[-1])
        return simplified


# ============================================================
# Layer 2: 局部轨迹规划器 (Trajectory Generator)
# ============================================================
class TrajectoryGenerator:
    """
    轨迹生成器 - 将几何路径转换为时间参数化轨迹
    使用平滑的曲线插值，边走边转，不停顿
    """
    
    def __init__(self, v_max=0.20, v_min=0.05, w_max=1.5, a_max=0.5):
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max
        self.a_max = a_max
        
        rospy.loginfo(f"[Layer2-TrajGen] Smooth Trajectory Generator initialized")
    
    def generate(self, geometric_path, current_state, dt=0.1):
        """
        平滑轨迹生成 - 边走边转，不停顿
        """
        if geometric_path is None or len(geometric_path) < 2:
            return None
        
        # 1. 首先对路径进行密集采样（插值）
        dense_path = self._interpolate_path(geometric_path, spacing=0.05)
        
        # 2. 计算每个点的曲率，用于速度规划
        curvatures = self._compute_curvatures(dense_path)
        
        # 3. 基于曲率的速度规划
        speeds = self._plan_speeds(dense_path, curvatures)
        
        # 4. 生成时间参数化轨迹
        trajectory = []
        t = 0.0
        
        for i in range(len(dense_path)):
            x, y = dense_path[i]
            v = speeds[i]
            
            # 计算朝向（看向下一个点）
            if i < len(dense_path) - 1:
                dx = dense_path[i+1][0] - x
                dy = dense_path[i+1][1] - y
                theta = np.arctan2(dy, dx)
            else:
                theta = trajectory[-1]['theta'] if trajectory else current_state[2]
            
            # 计算角速度
            if i > 0:
                dtheta = self._normalize_angle(theta - trajectory[-1]['theta'])
                omega = dtheta / dt if dt > 0 else 0.0
                omega = np.clip(omega, -self.w_max, self.w_max)
            else:
                omega = 0.0
            
            trajectory.append({
                'x': x, 'y': y, 'theta': theta,
                'v': v, 'omega': omega, 't': t
            })
            
            # 时间步进：基于速度和距离
            if i < len(dense_path) - 1 and v > 0.01:
                dist_to_next = np.sqrt((dense_path[i+1][0] - x)**2 + 
                                       (dense_path[i+1][1] - y)**2)
                t += dist_to_next / max(v, 0.01)
            else:
                t += dt
        
        # 添加停止点
        for _ in range(10):
            trajectory.append({
                'x': trajectory[-1]['x'],
                'y': trajectory[-1]['y'],
                'theta': trajectory[-1]['theta'],
                'v': 0.0, 'omega': 0.0, 't': t
            })
            t += dt
        
        rospy.loginfo(f"[Layer2-TrajGen] Smooth trajectory: {len(trajectory)} pts, {t:.1f}s")
        return trajectory
    
    def _interpolate_path(self, path, spacing=0.05):
        """对路径进行线性插值，生成密集点"""
        if len(path) < 2:
            return path
        
        dense = [path[0]]
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i+1]
            dist = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            n_points = max(2, int(dist / spacing))
            
            for j in range(1, n_points + 1):
                alpha = j / n_points
                x = p1[0] + alpha * (p2[0] - p1[0])
                y = p1[1] + alpha * (p2[1] - p1[1])
                dense.append((x, y))
        
        return dense
    
    def _compute_curvatures(self, path):
        """计算路径上每点的曲率"""
        n = len(path)
        curvatures = [0.0] * n
        
        for i in range(1, n - 1):
            # 使用三点计算曲率
            p0, p1, p2 = path[i-1], path[i], path[i+1]
            
            # 向量
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            
            # 角度变化
            angle1 = np.arctan2(v1[1], v1[0])
            angle2 = np.arctan2(v2[1], v2[0])
            dangle = abs(self._normalize_angle(angle2 - angle1))
            
            # 弧长
            ds = (np.sqrt(v1[0]**2 + v1[1]**2) + np.sqrt(v2[0]**2 + v2[1]**2)) / 2
            
            if ds > 0.001:
                curvatures[i] = dangle / ds
            else:
                curvatures[i] = 0.0
        
        # 平滑曲率
        smoothed = curvatures.copy()
        for i in range(2, n - 2):
            smoothed[i] = np.mean(curvatures[i-2:i+3])
        
        return smoothed
    
    def _plan_speeds(self, path, curvatures):
        """基于曲率规划速度 - 弯道减速"""
        n = len(path)
        speeds = [self.v_max] * n
        
        # 1. 根据曲率调整速度（弯道减速）
        for i in range(n):
            # 曲率越大，速度越低
            # v = v_max / (1 + k * curvature)
            k = 2.0  # 曲率敏感度
            speeds[i] = self.v_max / (1 + k * curvatures[i])
            speeds[i] = max(self.v_min, min(self.v_max, speeds[i]))
        
        # 2. 终点减速
        decel_dist = 0.3  # 减速区域
        total_dist = 0
        for i in range(n - 1, 0, -1):
            dist = np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            total_dist += dist
            if total_dist < decel_dist:
                # 线性减速到0
                speeds[i] = min(speeds[i], self.v_max * (total_dist / decel_dist))
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
                speeds[i] = min(speeds[i], self.v_max * (total_dist / accel_dist) + self.v_min)
        
        return speeds
    
    def _normalize_angle(self, angle):
        """角度归一化到 [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_reference_at_time(self, trajectory, t):
        """
        获取时间 t 处的参考状态（线性插值）
        """
        if trajectory is None or len(trajectory) == 0:
            return None
        
        # 找到时间 t 对应的轨迹点
        for i in range(len(trajectory) - 1):
            if trajectory[i]['t'] <= t < trajectory[i+1]['t']:
                # 线性插值
                t0, t1 = trajectory[i]['t'], trajectory[i+1]['t']
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0
                
                ref = {}
                for key in ['x', 'y', 'theta', 'v', 'omega']:
                    ref[key] = trajectory[i][key] * (1 - alpha) + trajectory[i+1][key] * alpha
                ref['t'] = t
                return ref
        
        # 超出轨迹末尾，返回最后一个点
        return trajectory[-1]
    
    def get_reference_window(self, trajectory, t_start, horizon, dt):
        """
        获取从 t_start 开始的 horizon 个参考点
        用于 MPC 的参考轨迹
        """
        refs = []
        for i in range(horizon + 1):
            t = t_start + i * dt
            ref = self.get_reference_at_time(trajectory, t)
            if ref is not None:
                refs.append(ref)
            elif len(refs) > 0:
                refs.append(refs[-1])  # 复制最后一个
        return refs


# ============================================================
# Layer 3: 轨迹跟踪控制器 (MPC Tracker)
# ============================================================
class MPCTracker:
    """
    MPC 轨迹跟踪控制器 - 平滑跟踪
    """
    
    def __init__(self, dt=0.1, N=20):
        self.dt = dt
        self.N = N  # 预测步数（增加以获得更平滑的轨迹）
        
        # 控制约束
        self.v_max = 0.22
        self.v_min = -0.05
        self.w_max = 1.8
        
        # 代价权重 - 调整为更平滑
        self.Q_x = 15.0      # 位置x跟踪
        self.Q_y = 15.0      # 位置y跟踪
        self.Q_theta = 5.0   # 朝向跟踪（增加）
        self.Q_v = 3.0       # 速度跟踪（增加，更好地跟踪参考速度）
        self.R_v = 0.05      # 控制量v（减小）
        self.R_w = 0.05      # 控制量omega（减小）
        self.R_dv = 1.0      # 控制变化率（增加，更平滑）
        self.R_dw = 1.5      # 角速度变化率（增加，更平滑）
        
        self._setup_mpc()
        
        rospy.loginfo(f"[Layer3-MPC] Smooth Tracker initialized: horizon={N}, dt={dt}")
    
    def _setup_mpc(self):
        """构建 MPC 问题"""
        # 状态: [x, y, theta]
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        state = ca.vertcat(x, y, theta)
        n_state = 3
        
        # 控制: [v, omega]
        v = ca.SX.sym('v')
        w = ca.SX.sym('w')
        control = ca.vertcat(v, w)
        n_control = 2
        
        # 动力学
        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), w)
        self.f = ca.Function('f', [state, control], [rhs])
        
        # 优化变量
        X = ca.SX.sym('X', n_state, self.N + 1)
        U = ca.SX.sym('U', n_control, self.N)
        
        # 参数: 初始状态(3) + 参考轨迹(N+1个点×4维) + 上一次控制(2)
        # 参考点: [x_ref, y_ref, theta_ref, v_ref]
        n_ref_per_point = 4
        P = ca.SX.sym('P', n_state + (self.N + 1) * n_ref_per_point + n_control)
        
        obj = 0
        g = []
        
        # 初始状态约束
        g.append(X[:, 0] - P[:n_state])
        
        # 提取上一次控制
        v_last = P[n_state + (self.N + 1) * n_ref_per_point]
        w_last = P[n_state + (self.N + 1) * n_ref_per_point + 1]
        
        for k in range(self.N):
            # 提取参考点
            ref_start = n_state + k * n_ref_per_point
            x_ref = P[ref_start]
            y_ref = P[ref_start + 1]
            theta_ref = P[ref_start + 2]
            v_ref = P[ref_start + 3]
            
            # 当前状态
            x_k, y_k, theta_k = X[0, k], X[1, k], X[2, k]
            
            # 跟踪误差代价
            obj += self.Q_x * (x_k - x_ref)**2
            obj += self.Q_y * (y_k - y_ref)**2
            
            # 朝向误差（考虑角度周期性）
            theta_err = ca.atan2(ca.sin(theta_k - theta_ref), ca.cos(theta_k - theta_ref))
            obj += self.Q_theta * theta_err**2
            
            # 速度跟踪
            obj += self.Q_v * (U[0, k] - v_ref)**2
            
            # 控制代价
            obj += self.R_v * U[0, k]**2
            obj += self.R_w * U[1, k]**2
            
            # 控制变化率
            if k == 0:
                dv = U[0, k] - v_last
                dw = U[1, k] - w_last
            else:
                dv = U[0, k] - U[0, k-1]
                dw = U[1, k] - U[1, k-1]
            obj += self.R_dv * dv**2 + self.R_dw * dw**2
            
            # 动力学约束
            x_next = X[:, k] + self.dt * self.f(X[:, k], U[:, k])
            g.append(X[:, k+1] - x_next)
        
        # 终端代价
        ref_start = n_state + self.N * n_ref_per_point
        x_ref_N = P[ref_start]
        y_ref_N = P[ref_start + 1]
        theta_ref_N = P[ref_start + 2]
        
        obj += self.Q_x * 3.0 * (X[0, self.N] - x_ref_N)**2
        obj += self.Q_y * 3.0 * (X[1, self.N] - y_ref_N)**2
        theta_err_N = ca.atan2(ca.sin(X[2, self.N] - theta_ref_N), 
                               ca.cos(X[2, self.N] - theta_ref_N))
        obj += self.Q_theta * 3.0 * theta_err_N**2
        
        # 构建 NLP
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
        self.n_ref_per_point = n_ref_per_point
        
        # 约束边界
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
        
        # Warm start
        self.u0 = np.zeros((self.N, self.n_control))
        self.x0 = np.zeros((self.N + 1, self.n_state))
        self.last_control = np.array([0.0, 0.0])
    
    def solve(self, current_state, reference_window):
        """
        求解 MPC
        
        输入:
            current_state: [x, y, theta]
            reference_window: list of dict [{x, y, theta, v, omega, t}, ...]
            
        输出:
            control: [v, omega]
            predicted_trajectory: [[x, y, theta], ...]
        """
        if reference_window is None or len(reference_window) < self.N + 1:
            rospy.logwarn("[Layer3-MPC] Insufficient reference points!")
            return self.last_control, None
        
        # 构建参数向量
        p = list(current_state)
        
        # 添加参考轨迹
        for i in range(self.N + 1):
            ref = reference_window[i] if i < len(reference_window) else reference_window[-1]
            p.extend([ref['x'], ref['y'], ref['theta'], ref['v']])
        
        # 添加上一次控制
        p.extend(self.last_control.tolist())
        
        p = np.array(p)
        
        # 初始猜测
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
            
            # Warm start 更新
            self.x0[:-1, :] = x_opt[1:, :]
            self.x0[-1, :] = x_opt[-1, :]
            self.u0[:-1, :] = u_opt[1:, :]
            self.u0[-1, :] = u_opt[-1, :]
            
            self.last_control = u_opt[0]
            
            return u_opt[0], x_opt
            
        except Exception as e:
            rospy.logwarn(f"[Layer3-MPC] Solve failed: {e}")
            return self.last_control, None
    
    def reset(self):
        """重置控制器状态"""
        self.u0 = np.zeros((self.N, self.n_control))
        self.x0 = np.zeros((self.N + 1, self.n_state))
        self.last_control = np.array([0.0, 0.0])


# ============================================================
# 主控制器：整合三层架构
# ============================================================
class ThreeLayerController:
    def __init__(self):
        rospy.init_node('three_layer_controller')
        rospy.loginfo("=" * 60)
        rospy.loginfo("Three-Layer Hierarchical Controller")
        rospy.loginfo("  Layer 1: Global Path Planning (A*)")
        rospy.loginfo("  Layer 2: Trajectory Generation (Time-parameterized)")
        rospy.loginfo("  Layer 3: Trajectory Tracking (MPC)")
        rospy.loginfo("=" * 60)
        
        # 状态变量
        self.goal_pos = None
        self.current_state = None
        self.obstacle_points = []
        
        # 三层模块 - 参数调整为更平滑
        self.global_planner = GlobalPathPlanner(resolution=0.05)
        self.traj_generator = TrajectoryGenerator(v_max=0.18, v_min=0.08, w_max=1.2)
        self.mpc_tracker = MPCTracker(dt=0.1, N=20)
        
        # 规划结果
        self.geometric_path = []
        self.reference_trajectory = None
        self.traj_start_time = None
        
        # 重规划参数
        self.replan_interval = 3.0  # 减少重规划频率
        self.last_replan_time = 0
        
        # ROS 接口
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.pub_ref_traj = rospy.Publisher('/reference_trajectory', Path, queue_size=1)
        self.pub_mpc_traj = rospy.Publisher('/mpc_predict_path', Path, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.pub_waypoint_marker = rospy.Publisher('/current_ref_marker', Marker, queue_size=1)
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        
        self.map_received = False
        
        # 控制循环
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        rospy.loginfo("[Controller] Ready. Waiting for map and goal...")
    
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_state = np.array([x, y, yaw])
    
    def goal_cb(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.geometric_path = []
        self.reference_trajectory = None
        self.traj_start_time = None
        self.last_replan_time = 0
        self.mpc_tracker.reset()
        
        rospy.loginfo(f"[Controller] New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")
    
    def scan_cb(self, msg):
        if self.current_state is None:
            return
        x, y, theta = self.current_state
        obstacle_points = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 3 == 0 and msg.range_min < r < msg.range_max and r < 3.0:
                obs_x = x + r * np.cos(theta + angle)
                obs_y = y + r * np.sin(theta + angle)
                obstacle_points.append((obs_x, obs_y))
            angle += msg.angle_increment
        self.obstacle_points = obstacle_points
    
    def map_cb(self, msg):
        if not self.map_received:
            rospy.loginfo(f"[Controller] Map received: {msg.info.width}x{msg.info.height}")
        self.global_planner.load_static_map(msg)
        self.map_received = True
    
    def control_loop(self, event):
        if self.current_state is None:
            return
        
        if self.goal_pos is None:
            self.pub_cmd.publish(Twist())
            return
        
        # 检查是否到达目标
        dist_to_goal = np.linalg.norm(self.goal_pos - self.current_state[:2])
        if dist_to_goal < 0.15:
            rospy.loginfo("[Controller] Goal reached!")
            self.goal_pos = None
            self.reference_trajectory = None
            self.pub_cmd.publish(Twist())
            return
        
        # === Layer 1 & 2: 定期重规划 ===
        current_time = rospy.Time.now().to_sec()
        need_replan = (current_time - self.last_replan_time > self.replan_interval or 
                       self.reference_trajectory is None)
        
        if need_replan:
            self._do_planning()
            self.last_replan_time = current_time
        
        if self.reference_trajectory is None:
            self.pub_cmd.publish(Twist())
            return
        
        # === Layer 3: MPC 轨迹跟踪 ===
        # 计算当前在轨迹上的时间
        elapsed = current_time - self.traj_start_time
        
        # 获取参考窗口
        ref_window = self.traj_generator.get_reference_window(
            self.reference_trajectory, elapsed, self.mpc_tracker.N, self.dt)
        
        if ref_window is None or len(ref_window) < self.mpc_tracker.N + 1:
            # 轨迹结束，重新规划
            self._do_planning()
            self.last_replan_time = current_time
            return
        
        # MPC 求解
        control, mpc_traj = self.mpc_tracker.solve(self.current_state, ref_window)
        
        # 发布控制命令
        cmd = Twist()
        cmd.linear.x = float(control[0])
        cmd.angular.z = float(control[1])
        self.pub_cmd.publish(cmd)
        
        # 可视化
        self._visualize(ref_window, mpc_traj)
        
        rospy.loginfo_throttle(1.0, 
            f"[Controller] dist={dist_to_goal:.2f}m, v={control[0]:.2f}, w={control[1]:.2f}")
    
    def _do_planning(self):
        """执行 Layer 1 + Layer 2 规划"""
        if not self.map_received or self.goal_pos is None:
            return
        
        # Layer 1: 全局路径规划
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
        rospy.loginfo(f"[Controller] Planning complete: path={len(path)} pts, "
                      f"traj={len(self.reference_trajectory)} pts")
    
    def _visualize(self, ref_window, mpc_traj):
        """可视化"""
        now = rospy.Time.now()
        
        # 全局几何路径 (绿色)
        if len(self.geometric_path) > 0:
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = now
            for wp in self.geometric_path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.pub_global_path.publish(path_msg)
        
        # 参考轨迹 (蓝色)
        if self.reference_trajectory is not None:
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = now
            for pt in self.reference_trajectory[::3]:  # 降采样显示
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = pt['x']
                pose.pose.position.y = pt['y']
                q = quaternion_from_euler(0, 0, pt['theta'])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                path_msg.poses.append(pose)
            self.pub_ref_traj.publish(path_msg)
        
        # MPC 预测轨迹 (红色)
        if mpc_traj is not None:
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = now
            for state in mpc_traj:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = state[0]
                pose.pose.position.y = state[1]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.pub_mpc_traj.publish(path_msg)
        
        # 目标点
        if self.goal_pos is not None:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = now
            marker.ns = "goal"
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.goal_pos[0]
            marker.pose.position.y = self.goal_pos[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.3
            marker.scale.z = 0.2
            marker.color.a = 0.8
            marker.color.g = 1.0
            self.pub_goal_marker.publish(marker)
        
        # 当前参考点
        if ref_window and len(ref_window) > 0:
            ref = ref_window[0]
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = now
            marker.ns = "current_ref"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = ref['x']
            marker.pose.position.y = ref['y']
            marker.pose.position.z = 0.1
            q = quaternion_from_euler(0, 0, ref['theta'])
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.scale.x = 0.3
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.pub_waypoint_marker.publish(marker)


if __name__ == '__main__':
    try:
        ThreeLayerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
