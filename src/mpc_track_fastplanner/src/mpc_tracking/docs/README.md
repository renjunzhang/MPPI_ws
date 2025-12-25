# MPC + FastPlanner Launch Files

统一的launch文件支持实物平台和Gazebo仿真的灵活切换。

## 快速开始

### 仿真模式（默认）
```bash
roslaunch mpc_tracking unified.launch mode:=sim
```

### 实物模式（hf_platform）
```bash
roslaunch mpc_tracking unified.launch mode:=real
```

### 调试模式（降速）
```bash
# 仿真 + 调试
roslaunch mpc_tracking unified.launch mode:=sim debug:=true

# 实物 + 调试
roslaunch mpc_tracking unified.launch mode:=real debug:=true
```

---

## 话题映射对比

| 组件 | 参数名 | 仿真模式 | 实物模式 |
|------|--------|---------|---------|
| 激光雷达 | `scan_topic` | `/front/scan` | `/scan_full_filtered` |
| 速度命令输出 | `cmd_vel_topic` | `/cmd_vel` | `/mpc/cmd_vel_raw` |
| 里程计 | `odom_topic` | `/odom` | `/odom` |
| 点云转换后 | `cloud_topic` | `/point_cloud_map` | `/point_cloud_map` |

---

## 模块启动情况

### 所有模式都会启动：
1. **Lidar2World** - 激光 → 点云坐标转换
2. **FastPlanner** - 全局路径规划（KinoReplanFSM + TopoReplanFSM）
3. **Waypoint Generator** - 航点生成（RViz 2D Nav Goal）
4. **MPC Tracking Controller** - 轨迹跟踪控制
5. **RViz** - 可视化界面

### 仅实物模式启动：
- **Safety Switch Node** - 紧急停止 (B键)

### 可选启用：
- **Trajectory Server** - 从B样条轨迹生成速度指令（默认关闭）
  ```bash
  roslaunch mpc_tracking unified.launch mode:=sim use_traj_server:=true
  ```

---

## 参数调整

### 速度限制
```bash
# 仿真，限速 0.5 m/s，加速度 0.4 m/s²
roslaunch mpc_tracking unified.launch mode:=sim max_vel:=0.5 max_acc:=0.4
```

### 地图尺寸
```bash
# 40m × 40m × 0.5m (默认)
roslaunch mpc_tracking unified.launch map_size_x:=50.0 map_size_y:=50.0
```

### MPC参数
编辑launch文件中的以下参数：
- `dt` - 控制周期（默认0.1s）
- `N` - 预测步数（默认60）
- `w_pos` - 位置权重（默认10.0）
- `w_yaw` - 航向权重（默认5.0）
- `w_u` - 控制输入权重（默认2.0）
- `w_du` - 控制平滑性权重（默认50.0，调试时10.0）

---

## 实物与仿真主要差异

### Gazebo 仿真环境
- 激光话题: `/front/scan` (LaserScan)
- 模拟时钟: `/clock` (ROS bag or Gazebo)
- 速度输出: `/cmd_vel`（直接给底层控制）
- 无安全开关

### 实物平台 (hf_platform)
- 激光话题: `/scan_full_filtered` (LaserScan，经过滤波)
- 系统时钟: 实时系统时间
- 速度输出: `/mpc/cmd_vel_raw`（经过安全开关，B键急停）
- 安全机制: Safety Switch Node（控制紧急停止）

---

## 通讯链路

### 仿真模式
```
RViz 2D Nav Goal
    ↓
Waypoint Generator
    ↓ /waypoint_generator/waypoints
    ↓
FastPlanner
    ↓ /planning/bspline
    ↓
MPC Tracking Controller
    ↓ /cmd_vel
    ↓
Gazebo (底层控制)
```

### 实物模式
```
RViz 2D Nav Goal
    ↓
Waypoint Generator
    ↓ /waypoint_generator/waypoints
    ↓
FastPlanner
    ↓ /planning/bspline
    ↓
MPC Tracking Controller
    ↓ /mpc/cmd_vel_raw
    ↓
Safety Switch (B键急停)
    ↓ /hf_platform/twist_mux/cmd_vel
    ↓
hf_platform 底层控制
```

---

## 故障排除

### LaserScan 转 PointCloud2 失败
- 检查激光话题名称是否正确（根据mode自动映射）
- 确保TF树已建立 (`rostf tf_monitor`)

### 无法订阅到 `/planning/bspline`
- 确保FastPlanner正确启动：`rostopic echo /planning/bspline`
- 检查是否在RViz中设置了目标点（2D Nav Goal）

### MPC不输出速度命令
- 检查MPC控制器是否收到轨迹：`rostopic echo /planning/bspline`
- 检查里程计是否正常：`rostopic echo /odom`

### 实物模式安全开关不工作
- 检查手柄连接：`rostopic echo /joy`
- 查看Safety Switch日志：`rosrun rqt_console rqt_console`

---

## 原始Launch 文件参考

- `hf_platform.launch` - 实物平台原始版本
- `hf_platform_debug.launch` - 实物平台调试版本（降速）
- `unified.launch` - 统一版本（推荐使用）

---

## 相关文件位置

| 文件 | 路径 |
|------|------|
| Unified Launch | `src/mpc_tracking/launch/unified.launch` |
| MPC 节点 | `src/mpc_tracking/src/mpc_node.cpp` |
| FastPlanner | `src/Fast-Planner/fast_planner/plan_manage/` |
| 点云转换 | `src/lidar2world/` |
