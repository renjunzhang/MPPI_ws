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


# ==================== 测试代码 ====================

def _test():
    """单元测试"""
    print("=" * 50)
    print("GlobalPathPlanner 测试")
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
