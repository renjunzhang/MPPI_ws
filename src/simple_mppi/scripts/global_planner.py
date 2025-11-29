#!/usr/bin/env python3
"""
全局路径规划模块 (Layer 1)
==========================
基于 A* 算法的栅格地图全局路径规划

功能:
    - 加载 OccupancyGrid 地图
    - 障碍物膨胀处理
    - A* 路径搜索
    - 路径简化
    - 动态障碍物更新

输入: 起点坐标, 终点坐标
输出: 几何路径 [(x, y), ...]

作者: GitHub Copilot
"""

import numpy as np
import heapq

# ROS 依赖 (可选)
try:
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


def _log_info(msg):
    """日志输出 (兼容 ROS 和纯 Python 环境)"""
    if HAS_ROS and not rospy.is_shutdown():
        rospy.loginfo(msg)
    else:
        print(f"[INFO] {msg}")


def _log_warn(msg):
    """警告输出"""
    if HAS_ROS and not rospy.is_shutdown():
        rospy.logwarn(msg)
    else:
        print(f"[WARN] {msg}")


class GlobalPathPlanner:
    """
    A* 全局路径规划器
    
    将栅格地图上的路径规划问题转化为图搜索问题，
    使用 A* 算法寻找从起点到终点的最短路径。
    
    Attributes:
        resolution (float): 栅格分辨率 (m/cell)
        inflate_radius (float): 障碍物膨胀半径 (m)
        grid (np.ndarray): 当前栅格地图 (含动态障碍物)
        has_static_map (bool): 是否已加载静态地图
    """
    
    def __init__(self, resolution=0.05, inflate_radius=0.25):
        """
        初始化规划器
        
        Args:
            resolution: 栅格分辨率 (m/cell)
            inflate_radius: 障碍物膨胀半径 (m)，应略大于机器人半径
        """
        self.resolution = resolution
        self.inflate_radius = inflate_radius
        self.inflate_cells = int(inflate_radius / resolution)
        
        # 地图数据
        self.static_grid = None
        self.grid = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width = 0
        self.height = 0
        self.has_static_map = False
        
        _log_info("[GlobalPlanner] Initialized")
    
    # ==================== 地图处理 ====================
    
    def load_static_map(self, map_msg):
        """
        加载 ROS OccupancyGrid 地图
        
        Args:
            map_msg: nav_msgs/OccupancyGrid 消息
        """
        self.width = map_msg.info.width
        self.height = map_msg.info.height
        self.resolution = map_msg.info.resolution
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y
        
        # 解析地图: 值 > 50 或 < 0 视为障碍物
        map_data = np.array(map_msg.data).reshape((self.height, self.width))
        self.static_grid = np.zeros((self.width, self.height), dtype=np.uint8)
        self.static_grid[map_data.T > 50] = 1
        self.static_grid[map_data.T < 0] = 1
        
        # 膨胀障碍物
        self.inflate_cells = int(self.inflate_radius / self.resolution)
        self.static_grid = self._inflate_obstacles(self.static_grid)
        self.grid = self.static_grid.copy()
        self.has_static_map = True
        
        _log_info(f"[GlobalPlanner] Map: {self.width}x{self.height}, res={self.resolution:.3f}m")
    
    def load_grid_map(self, grid, resolution, origin=(0, 0)):
        """
        直接加载栅格地图数组 (用于测试)
        
        Args:
            grid: 2D numpy 数组, 1=障碍物, 0=自由
            resolution: 分辨率 (m/cell)
            origin: 地图原点 (x, y)
        """
        self.static_grid = self._inflate_obstacles(grid.astype(np.uint8))
        self.grid = self.static_grid.copy()
        self.resolution = resolution
        self.origin_x, self.origin_y = origin
        self.width, self.height = grid.shape
        self.has_static_map = True
    
    def _inflate_obstacles(self, grid):
        """膨胀障碍物"""
        from scipy import ndimage
        
        kernel_size = 2 * self.inflate_cells + 1
        kernel = np.zeros((kernel_size, kernel_size))
        for i in range(kernel_size):
            for j in range(kernel_size):
                if (i - self.inflate_cells)**2 + (j - self.inflate_cells)**2 <= self.inflate_cells**2:
                    kernel[i, j] = 1
        
        return ndimage.binary_dilation(grid, structure=kernel).astype(np.uint8)
    
    def update_dynamic_obstacles(self, obstacle_points):
        """
        更新动态障碍物
        
        Args:
            obstacle_points: 障碍物坐标列表 [(x, y), ...]
        """
        if not self.has_static_map:
            return
        
        self.grid = self.static_grid.copy()
        
        for ox, oy in obstacle_points:
            gx, gy = self.world_to_grid(ox, oy)
            if not self._in_bounds(gx, gy):
                continue
            
            for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                for dy in range(-self.inflate_cells, self.inflate_cells + 1):
                    if dx*dx + dy*dy <= self.inflate_cells**2:
                        nx, ny = gx + dx, gy + dy
                        if self._in_bounds(nx, ny):
                            self.grid[nx, ny] = 1
    
    # ==================== 坐标转换 ====================
    
    def world_to_grid(self, x, y):
        """世界坐标 -> 栅格坐标"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """栅格坐标 -> 世界坐标"""
        x = gx * self.resolution + self.origin_x + self.resolution / 2
        y = gy * self.resolution + self.origin_y + self.resolution / 2
        return x, y
    
    def _in_bounds(self, gx, gy):
        """检查是否在地图范围内"""
        return 0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]
    
    # ==================== A* 搜索 ====================
    
    def plan(self, start_xy, goal_xy):
        """
        A* 路径规划
        
        Args:
            start_xy: 起点 (x, y)
            goal_xy: 终点 (x, y)
            
        Returns:
            路径 [(x, y), ...] 或 None
        """
        if self.grid is None:
            _log_warn("[GlobalPlanner] No map!")
            return None
        
        start = self.world_to_grid(start_xy[0], start_xy[1])
        goal = self.world_to_grid(goal_xy[0], goal_xy[1])
        
        # 边界检查
        if not self._in_bounds(start[0], start[1]):
            _log_warn(f"[GlobalPlanner] Start out of bounds")
            return None
        if not self._in_bounds(goal[0], goal[1]):
            _log_warn(f"[GlobalPlanner] Goal out of bounds")
            return None
        
        # 处理障碍物中的起点/终点
        if self.grid[start[0], start[1]] == 1:
            start = self._find_nearest_free(start)
            if start is None:
                return None
        
        if self.grid[goal[0], goal[1]] == 1:
            goal = self._find_nearest_free(goal)
            if goal is None:
                return None
        
        # A* 搜索
        path = self._astar_search(start, goal)
        if path is None:
            _log_warn("[GlobalPlanner] No path found!")
            return None
        
        simplified = self._simplify_path(path)
        _log_info(f"[GlobalPlanner] Path: {len(simplified)} waypoints")
        return simplified
    
    def _astar_search(self, start, goal, max_iter=50000):
        """A* 搜索核心"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        
        for _ in range(max_iter):
            if not open_set:
                break
            
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                return self._reconstruct_path(came_from, start, current)
            
            for neighbor, cost in self._get_neighbors(current):
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = np.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
        
        return None
    
    def _get_neighbors(self, node):
        """获取8邻域"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = node[0] + dx, node[1] + dy
                if self._in_bounds(nx, ny) and self.grid[nx, ny] == 0:
                    cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                    neighbors.append(((nx, ny), cost))
        return neighbors
    
    def _reconstruct_path(self, came_from, start, current):
        """重建路径"""
        path = []
        while current in came_from:
            path.append(self.grid_to_world(current[0], current[1]))
            current = came_from[current]
        path.append(self.grid_to_world(start[0], start[1]))
        path.reverse()
        return path
    
    def _find_nearest_free(self, point, max_radius=30):
        """寻找最近自由栅格"""
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = point[0] + dx, point[1] + dy
                        if self._in_bounds(nx, ny) and self.grid[nx, ny] == 0:
                            return (nx, ny)
        return None
    
    def _simplify_path(self, path, tolerance=0.08):
        """简化路径"""
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
            if len1 > 0 and len2 > 0 and cross / (len1 * len2) > tolerance:
                simplified.append(p1)
        simplified.append(path[-1])
        return simplified
    
    def is_path_valid(self, path):
        """检查路径有效性"""
        if path is None or len(path) < 2:
            return False
        
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            dist = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            n_checks = max(2, int(dist / self.resolution))
            
            for j in range(n_checks + 1):
                alpha = j / n_checks
                x = p1[0] + alpha * (p2[0] - p1[0])
                y = p1[1] + alpha * (p2[1] - p1[1])
                gx, gy = self.world_to_grid(x, y)
                if not self._in_bounds(gx, gy) or self.grid[gx, gy] == 1:
                    return False
        return True


# ==================== 测试/交互代码 ====================

def _test():
    """增强的测试：
    - 如果运行在 ROS 环境，会尝试从 `turtlebot3_navigation` 加载 `maps/map.yaml`，
      订阅 `/odom` 获取起点，订阅 `/move_base_simple/goal`（RViz 的 2D Nav Goal）作为目标，
      规划后发布 `nav_msgs/Path` 到 `/planned_path`，并在控制台打印信息。
    - 如果没有 ROS，会回退到原先的简单栅格单元测试。
    """
    if HAS_ROS:
        try:
            import rospy
            import rospkg
            import yaml
            from nav_msgs.msg import Path
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
        except Exception as e:
            print(f"[WARN] 需要 ROS 包 (rospkg/yaml/nav_msgs)，回退到离线测试: {e}")
            _offline_test()
            return

        class PlannerNode:
            def __init__(self):
                rospy.init_node('global_planner_test', anonymous=True)
                self.planner = GlobalPathPlanner()
                self.odom_pose = None
                self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

                # 订阅里程计以获得当前位姿
                rospy.Subscriber('/odom', Odometry, self._odom_cb)
                # RViz: 2D Nav Goal 发布到 /move_base_simple/goal
                rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_cb)

                # 尝试加载地图 yaml
                try:
                    rospack = rospkg.RosPack()
                    pkg_path = rospack.get_path('turtlebot3_navigation')
                    yaml_path = pkg_path + '/maps/map.yaml'
                    print(f"[INFO] Loading map from: {yaml_path}")
                    self._load_map_from_yaml(yaml_path)
                except Exception as e:
                    rospy.logwarn(f"无法加载 turtlebot3_navigation 地图: {e}")
                    rospy.logwarn("请确保已安装并包含该包，或手动调用 planner.load_grid_map()")

                rospy.loginfo('[GlobalPlannerTest] Ready. 在 RViz 使用 2D Nav Goal 发布目标到 /move_base_simple/goal')

            def _odom_cb(self, msg: Odometry):
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                # yaw
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = np.arctan2(siny, cosy)
                self.odom_pose = (px, py, yaw)

            def _goal_cb(self, msg: PoseStamped):
                gx = msg.pose.position.x
                gy = msg.pose.position.y
                # 选择起点：优先使用里程计，其次使用地图原点（更贴近实际地图坐标）
                if self.odom_pose is None:
                    rospy.logwarn('[GlobalPlannerTest] 无里程计数据，使用地图中心作为起点（建议用 AMCL 或 RViz 的 2D Pose Estimate 设置机器人位姿）')
                    # 使用地图中心作为更合理的默认起点，避免使用地图左下角原点导致不可达
                    map_cx = self.planner.origin_x + (self.planner.width * self.planner.resolution) / 2.0
                    map_cy = self.planner.origin_y + (self.planner.height * self.planner.resolution) / 2.0
                    start = (map_cx, map_cy)
                else:
                    start = (self.odom_pose[0], self.odom_pose[1])

                rospy.loginfo(f"规划: start={start} -> goal=({gx:.2f},{gy:.2f})")

                # 打印并检查网格索引与占用情况，便于诊断为何规划失败
                try:
                    sgx, sgy = self.planner.world_to_grid(start[0], start[1])
                    ggx, ggy = self.planner.world_to_grid(gx, gy)
                except Exception:
                    rospy.logwarn('[GlobalPlannerTest] 坐标转换失败（world_to_grid）')
                    sgx = sgy = ggx = ggy = None

                if sgx is not None:
                    rospy.loginfo(f"Start grid idx: ({sgx},{sgy}), Goal grid idx: ({ggx},{ggy})")
                    # 边界/占用检查
                    if not self.planner._in_bounds(ggx, ggy):
                        rospy.logwarn(f"[GlobalPlannerTest] 目标点超出地图范围: ({ggx},{ggy})")
                        return
                    if self.planner.grid[ggx, ggy] == 1:
                        rospy.logwarn('[GlobalPlannerTest] 目标点在障碍物上，尝试寻找最近自由点...')
                        nearest = self.planner._find_nearest_free((ggx, ggy), max_radius=50)
                        if nearest is not None:
                            ngx, ngy = nearest
                            ngw = self.planner.grid_to_world(ngx, ngy)
                            rospy.loginfo(f"找到可用替代目标: grid=({ngx},{ngy}) -> world={ngw}")
                            gx, gy = ngw
                        else:
                            rospy.logwarn('[GlobalPlannerTest] 未找到可替代目标，放弃规划')
                            return

                path = self.planner.plan(start, (gx, gy))
                if path is None:
                    rospy.logwarn('[GlobalPlannerTest] 未找到路径')
                    # 额外诊断信息
                    try:
                        rospy.loginfo(f"地图大小: width={self.planner.width}, height={self.planner.height}, res={self.planner.resolution}")
                        rospy.loginfo(f"起点网格: ({sgx},{sgy}) 占用={self.planner.grid[sgx, sgy] if sgx is not None else 'N/A'}")
                        rospy.loginfo(f"目标网格: ({ggx},{ggy}) 占用={self.planner.grid[ggx, ggy] if ggx is not None else 'N/A'}")
                    except Exception:
                        pass
                    return

                # 发布 nav_msgs/Path
                nav_path = Path()
                nav_path.header.stamp = rospy.Time.now()
                nav_path.header.frame_id = 'map'
                for (x, y) in path:
                    ps = PoseStamped()
                    ps.header = nav_path.header
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = 0.0
                    # 简单将朝向置为 0（Path 中方向信息不是必须的）
                    ps.pose.orientation.w = 1.0
                    nav_path.poses.append(ps)

                self.path_pub.publish(nav_path)
                rospy.loginfo(f"[GlobalPlannerTest] 发布路径: {len(path)} 点")

            def _load_map_from_yaml(self, yaml_path):
                import os
                from PIL import Image

                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)

                img_file = data.get('image')
                resolution = float(data.get('resolution', 0.05))
                origin = data.get('origin', [0.0, 0.0, 0.0])
                negate = int(data.get('negate', 0))
                occ_thresh = float(data.get('occupied_thresh', 0.65))

                img_path = os.path.join(os.path.dirname(yaml_path), img_file)
                im = Image.open(img_path).convert('L')
                arr = np.array(im).astype(np.float32)

                # 模仿 map_server 的处理逻辑：当 negate==0 时先做 255 - pixel
                if negate == 0:
                    img_vals = 255.0 - arr
                else:
                    img_vals = arr.copy()

                prob = img_vals / 255.0

                # 将像素概率映射到占用：prob > occupied_thresh -> 占用
                occ_mask = prob > occ_thresh

                grid = occ_mask.astype(np.uint8)
                # 转置以匹配 planner 内部的 (width, height) 约定
                grid_t = grid.T
                self.planner.load_grid_map(grid_t, resolution=resolution, origin=(origin[0], origin[1]))

        node = PlannerNode()
        rospy.spin()
    else:
        _offline_test()


def _offline_test():
    """原有的离线单元测试（非 ROS 环境回退）"""
    print("=" * 50)
    print("GlobalPathPlanner 离线测试")
    print("=" * 50)

    # 创建测试地图
    test_map = np.zeros((20, 20), dtype=np.uint8)
    test_map[5:15, 10] = 1  # 垂直墙

    planner = GlobalPathPlanner(resolution=0.1, inflate_radius=0.15)
    planner.load_grid_map(test_map, resolution=0.1)

    # 测试路径规划
    path = planner.plan((0.5, 0.5), (1.8, 1.5))

    if path:
        print(f"✓ 路径规划成功: {len(path)} 点")
        length = sum(np.sqrt((path[i+1][0]-path[i][0])**2 + 
                            (path[i+1][1]-path[i][1])**2) 
                    for i in range(len(path)-1))
        print(f"  路径长度: {length:.2f}m")
        print(f"  路径有效: {planner.is_path_valid(path)}")
    else:
        print("✗ 路径规划失败")

    print("=" * 50)


if __name__ == '__main__':
    _test()
