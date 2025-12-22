# mpc_track_fastplanner 迁移到 hf_platform 实物平台指南

## 概述

本指南说明如何将 **mpc_track_fastplanner** 仿真项目迁移到 **hf_platform** 实物平台运行。

### 系统架构对比

| 组件 | 仿真环境 | hf_platform 实物 |
|------|---------|-----------------|
| **底盘仿真** | omni_gazebo (Gazebo) | ❌ 不需要 |
| **激光雷达** | Velodyne (仿真) | sick_front + sick_rear (实物) |
| **里程计** | Gazebo `/odom` | hf_platform `/odom` ✅ 已有 |
| **控制输出** | `/cmd_vel` | `/hf_platform/twist_mux/cmd_vel` |
| **点云话题** | `/velodyne_points` | `/merged_cloud` 或 `/scan_full` |
| **定位** | Gazebo 真值 | AMCL ✅ 已有 |

---

## 核心修改点

### 1. MPC 跟踪控制器 (mpc_tracking)

**文件**: `/home/a/hf_move/src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp`

#### 修改 1.1：cmd_vel 话题重映射

**当前代码（仿真）**：
```cpp
cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
```

**修改后（实物）**：
```cpp
// 方案 A：直接修改代码
cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/hf_platform/twist_mux/cmd_vel", 1);

// 方案 B：使用参数（推荐）
std::string cmd_vel_topic;
nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
```

#### 修改 1.2：odom 话题确认

**当前代码**：
```cpp
ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomCallback);
```

**说明**：hf_platform 已有 `/odom` 话题，**无需修改**。但需要确认消息类型和坐标系：

```bash
# 检查 odom 话题
rostopic info /odom
rostopic echo /odom -n 1

# 确认坐标系
# header.frame_id 应为 "odom"
# child_frame_id 应为 "base_link" 或 "base_footprint"
```

---

### 2. 点云转换 (lidar2world)

**文件**: `/home/a/hf_move/src/mpc_track_fastplanner/src/lidar2world/src/transform.cpp`

#### 修改 2.1：激光雷达话题

**当前代码（仿真）**：
```cpp
ros::Subscriber points_sub = nh.subscribe("/velodyne_points", 10, pointCloudCallback);
```

**修改后（实物）**：
```cpp
// hf_platform 使用合并的激光点云
ros::Subscriber points_sub = nh.subscribe("/merged_cloud", 10, pointCloudCallback);
```

#### 修改 2.2：坐标系转换确认

**当前代码**：
```cpp
geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
```

**说明**：需要确认 hf_platform 的 TF 树结构：

```bash
# 查看 TF 树
rosrun tf view_frames
# 或
rosrun rqt_tf_tree rqt_tf_tree

# 检查激光雷达坐标系
rostopic echo /merged_cloud -n 1 | grep frame_id
# 可能是 "laser" 或 "sick_front" 等
```

**如果激光坐标系不是 `base_link`**，需要修改：
```cpp
// 假设激光坐标系为 "laser"
geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("odom", "laser", ros::Time(0));
```

#### 修改 2.3：处理 2D 激光扫描（可选）

如果使用 `/scan_full` (LaserScan) 而不是点云：

```cpp
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*msg, cloud);
    
    try {
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform("odom", msg->header.frame_id, ros::Time(0));
        
        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(cloud, transformed_cloud, transformStamped);
        
        transformed_cloud.header.frame_id = "odom";
        points_pub.publish(transformed_cloud);
    } catch(tf2::TransformException &e) {
        ROS_WARN("Failed to transform point cloud: %s", e.what());
    }
}

// 在 main 中
ros::Subscriber scan_sub = nh.subscribe("/scan_full", 10, laserScanCallback);
```

---

### 3. FastPlanner 配置

**文件**: `/home/a/hf_move/src/mpc_track_fastplanner/src/Fast-Planner/fast_planner/plan_manage/launch/16_lidar.launch`

#### 修改 3.1：地图尺寸调整

**当前配置（仿真）**：
```xml
<arg name="map_size_x" value="40.0"/>
<arg name="map_size_y" value="40.0"/>
<arg name="map_size_z" value="0.5"/>
```

**实物建议**：根据实际环境调整
```xml
<!-- 根据实际场地大小，例如 20m x 20m -->
<arg name="map_size_x" value="20.0"/>
<arg name="map_size_y" value="20.0"/>
<arg name="map_size_z" value="0.5"/>  <!-- 2D 导航，保持 0.5m 即可 -->
```

#### 修改 3.2：点云话题

**当前配置**：
```xml
<arg name="cloud_topic" value="/point_cloud_map"/>
```

**修改后**：
```xml
<!-- 使用 lidar2world 转换后的点云 -->
<arg name="cloud_topic" value="/point_cloud_map"/>
```

**说明**：这个话题由 `lidar2world` 发布，无需修改。

#### 修改 3.3：速度和加速度限制

**当前配置（仿真）**：
```xml
<arg name="max_vel" value="3" />
<arg name="max_acc" value="2" />
```

**实物建议**：根据平台性能调整
```xml
<!-- hf_platform 安全速度 -->
<arg name="max_vel" value="0.8" />  <!-- 0.5-1.0 m/s -->
<arg name="max_acc" value="0.5" />  <!-- 0.3-0.8 m/s² -->
```

---

## 修改后的代码文件

### 文件 1: mpc_node.cpp (修改版)

**位置**: `/home/a/hf_move/src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp`

**关键修改**：

```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_tracking_node");
    ros::NodeHandle nh("~");  // 使用私有命名空间

    // === 参数化配置（推荐） ===
    std::string cmd_vel_topic, odom_topic;
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/hf_platform/twist_mux/cmd_vel");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    
    // 发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 1);
   
    // 订阅器
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, &odomCallback);
    ros::Subscriber bspline_sub = nh.subscribe("/planning/bspline", 10, bsplineCallback);
    ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);

    control_cmd_pub = nh.createTimer(ros::Duration(0.1), publish_control_cmd);
    
    ROS_INFO("[MPC Tracking] Started with:");
    ROS_INFO("  cmd_vel_topic: %s", cmd_vel_topic.c_str());
    ROS_INFO("  odom_topic: %s", odom_topic.c_str());

    ros::spin();
    return 0;
}
```

---

### 文件 2: transform.cpp (修改版)

**位置**: `/home/a/hf_move/src/mpc_track_fastplanner/src/lidar2world/src/transform.cpp`

**关键修改**：

```cpp
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh("~");

    // === 参数化配置 ===
    std::string input_cloud_topic, output_cloud_topic;
    std::string target_frame, source_frame;
    
    nh.param<std::string>("input_cloud_topic", input_cloud_topic, "/merged_cloud");
    nh.param<std::string>("output_cloud_topic", output_cloud_topic, "/point_cloud_map");
    nh.param<std::string>("target_frame", target_frame, "odom");
    nh.param<std::string>("source_frame", source_frame, "base_link");  // 或自动检测
    
    ros::Subscriber points_sub = nh.subscribe(input_cloud_topic, 10, pointCloudCallback);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_topic, 10);
    
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    ROS_INFO("[lidar2world] Started with:");
    ROS_INFO("  input_cloud: %s", input_cloud_topic.c_str());
    ROS_INFO("  output_cloud: %s", output_cloud_topic.c_str());
    ROS_INFO("  target_frame: %s", target_frame.c_str());
    
    ros::spin();
    return 0;
}
```

**回调函数修改**：
```cpp
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    try {
        // 自动检测源坐标系
        std::string source_frame = msg->header.frame_id;
        
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform("odom", source_frame, ros::Time(0));

        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transformStamped);

        transformed_cloud.header.frame_id = "odom";
        points_pub.publish(transformed_cloud);
    } catch(tf2::TransformException &e) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform point cloud: %s", e.what());
    }
}
```

---

## 启动文件配置

### 创建实物平台 Launch 文件

**文件**: `/home/a/hf_move/src/mpc_track_fastplanner/launch/hf_platform.launch`

```xml
<launch>
  <!-- ========== 参数配置 ========== -->
  
  <!-- hf_platform 特定话题 -->
  <arg name="cmd_vel_topic" default="/hf_platform/twist_mux/cmd_vel"/>
  <arg name="odom_topic" default="/odom"/>
  <arg name="cloud_topic" default="/merged_cloud"/>
  
  <!-- FastPlanner 地图参数 -->
  <arg name="map_size_x" default="20.0"/>
  <arg name="map_size_y" default="20.0"/>
  <arg name="map_size_z" default="0.5"/>
  
  <!-- 速度限制（实物安全值） -->
  <arg name="max_vel" default="0.8"/>
  <arg name="max_acc" default="0.5"/>
  
  
  <!-- ========== 1. 点云转换节点 ========== -->
  <node pkg="lidar2world" name="lidar2world_node" type="lidar2world_node" output="screen">
    <param name="input_cloud_topic" value="$(arg cloud_topic)"/>
    <param name="output_cloud_topic" value="/point_cloud_map"/>
    <param name="target_frame" value="odom"/>
  </node>
  
  
  <!-- ========== 2. FastPlanner 规划器 ========== -->
  <include file="$(find plan_manage)/launch/lidar.xml">
    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="odometry_topic" value="$(arg odom_topic)"/>
    
    <arg name="cloud_topic" value="/point_cloud_map"/>
    
    <arg name="max_vel" value="$(arg max_vel)"/>
    <arg name="max_acc" value="$(arg max_acc)"/>
    
    <!-- 使用 2D Nav Goal 选择目标点 -->
    <arg name="flight_type" value="1"/>
  </include>
  
  
  <!-- ========== 3. 轨迹服务器 ========== -->
  <node pkg="plan_manage" name="traj_server" type="traj_server" output="screen">
    <remap from="/position_cmd" to="/planning/pos_cmd"/>
    <remap from="/odom_world" to="$(arg odom_topic)"/>
    <param name="traj_server/time_forward" value="1.5" type="double"/>
  </node>
  
  
  <!-- ========== 4. 航点生成器 ========== -->
  <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
    <remap from="~odom" to="$(arg odom_topic)"/>        
    <remap from="~goal" to="/move_base_simple/goal"/>
    <remap from="~traj_start_trigger" to="/traj_start_trigger"/>
    <param name="waypoint_type" value="manual-lonely-waypoint"/>    
  </node>
  
  
  <!-- ========== 5. MPC 跟踪控制器 ========== -->
  <node pkg="mpc_tracking" name="mpc_tracking_node" type="mpc_tracking_node" output="screen">
    <param name="cmd_vel_topic" value="$(arg cmd_vel_topic)"/>
    <param name="odom_topic" value="$(arg odom_topic)"/>
  </node>
  
  
  <!-- ========== 6. RViz 可视化 ========== -->
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find plan_manage)/launch/rviz_config/rviz.rviz"/>
  
</launch>
```

---

## 部署步骤

### Step 1: 代码修改

```bash
# 进入工作空间
cd /home/a/hf_move/src/mpc_track_fastplanner

# 修改 mpc_node.cpp（参考上面的修改）
nano src/mpc_tracking/src/mpc_node.cpp

# 修改 transform.cpp（参考上面的修改）
nano src/lidar2world/src/transform.cpp

# 创建实物启动文件
nano launch/hf_platform.launch
```

### Step 2: 编译

```bash
cd /home/a/hf_move
catkin_make --only-pkg-with-deps mpc_tracking lidar2world

# 或单独编译
catkin_make --pkg mpc_tracking
catkin_make --pkg lidar2world
```

### Step 3: 环境检查

使用 `junjun_bringup` 启动 hf_platform 后，检查必要话题：

```bash
# 检查 odom
rostopic hz /odom
rostopic echo /odom -n 1

# 检查激光雷达/点云
rostopic hz /merged_cloud
rostopic echo /merged_cloud -n 1 | grep frame_id
# 应该显示: frame_id: "scan_merge"

# 检查 TF 变换
rosrun tf view_frames
evince frames.pdf

# 确认 odom -> hf_base_link -> scan_merge 变换存在
rosrun tf tf_echo odom hf_base_link
rosrun tf tf_echo odom scan_merge
```

### Step 4: 启动系统

```bash
# 终端 1：启动 hf_platform
roslaunch junjun_bringup junjun_bringup.launch

# 终端 2：启动 mpc_track_fastplanner
roslaunch mpc_track_fastplanner hf_platform.launch

# 或使用快捷脚本：
/home/a/hf_move/src/mpc_track_fastplanner/launch/start_hf_platform.sh

# 终端 3（可选）：监控速度命令
rostopic echo /hf_platform/twist_mux/cmd_vel
```

### Step 5: 测试

在 RViz 中：

1. **设置 Fixed Frame**: `odom`
2. **添加显示**：
   - `/map` - 全局地图
   - `/point_cloud_map` - 局部障碍物
   - `/predict_path` - MPC 预测路径
   - `/planning/pos_cmd` - FastPlanner 轨迹
3. **发送目标点**：
   - 使用 "2D Nav Goal" 工具点击目标位置
   - 系统应该自动规划并执行

---

## 常见问题排查

### 问题 1: 点云不显示

**症状**：RViz 中看不到 `/point_cloud_map`

**排查步骤**：
```bash
# 检查输入点云
rostopic hz /merged_cloud

# 检查 TF 变换
rosrun tf tf_echo odom base_link
# 或
rosrun tf tf_echo odom laser

# 查看日志
rosnode info lidar2world_node
```

**解决方案**：
- 确认 `/merged_cloud` 正在发布
- 确认 TF 树完整（odom -> base_link 路径存在）
- 检查坐标系名称是否匹配

### 问题 2: 速度命令无效

**症状**：机器人不动，但 `/hf_platform/twist_mux/cmd_vel` 有数据

**排查步骤**：
```bash
# 检查 twist_mux 状态
rostopic list | grep twist_mux

# 查看速度命令优先级
rostopic echo /hf_platform/twist_mux/selected

# 检查是否有其他节点占用控制权
rostopic info /hf_platform/twist_mux/cmd_vel
```

**解决方案**：
- 确认 MPC 控制器在 twist_mux 的优先级列表中
- 可能需要禁用手柄或其他控制源
- 检查急停开关状态

### 问题 3: FastPlanner 规划失败

**症状**：点击目标点后无反应，或轨迹不合理

**排查步骤**：
```bash
# 检查 FastPlanner 状态
rosnode list | grep fast_planner
rosnode info /fast_planner_node

# 查看规划器日志
# （在启动的终端中）

# 检查地图构建
rostopic hz /sdf_map/occupancy_inflate
```

**解决方案**：
- 确认目标点在地图范围内
- 调整 `map_size_x/y/z` 参数
- 降低速度限制 `max_vel`
- 检查点云质量

### 问题 4: odom 坐标系不匹配

**症状**：机器人在 RViz 中位置错误或跳跃

**排查步骤**：
```bash
# 查看 odom 坐标系
rostopic echo /odom -n 1 | grep frame_id

# 检查 TF 树
rosrun rqt_tf_tree rqt_tf_tree
```

**解决方案**：
- 确认所有节点使用相同的坐标系名称
- 可能需要统一为 `odom` 或 `map`
- 检查 AMCL 是否正常工作

---

## 性能优化建议

### 1. 降低计算负载

```xml
<!-- 在 hf_platform.launch 中 -->

<!-- 降低点云分辨率 -->
<node pkg="lidar2world" name="lidar2world_node" type="lidar2world_node" output="screen">
  <param name="voxel_filter_size" value="0.1"/>  <!-- 下采样 -->
</node>

<!-- 降低规划频率 -->
<node pkg="plan_manage" ...>
  <param name="replan_frequency" value="5.0"/>  <!-- 从 10Hz 降到 5Hz -->
</node>

<!-- 降低 MPC 控制频率 -->
<node pkg="mpc_tracking" ...>
  <param name="control_frequency" value="10.0"/>  <!-- 从 10Hz -->
</node>
```

### 2. 调整 MPC 参数

根据实物平台动力学调整 MPC 参数（如果 mpc_tracking 支持）：

```xml
<node pkg="mpc_tracking" name="mpc_tracking_node" type="mpc_tracking_node" output="screen">
  <!-- 预测地平线 -->
  <param name="horizon" value="20"/>  <!-- 减少可提升实时性 -->
  
  <!-- 权重矩阵 -->
  <param name="q_pos" value="10.0"/>    <!-- 位置误差权重 -->
  <param name="r_vel" value="0.5"/>     <!-- 速度输出权重 -->
  
  <!-- 约束 -->
  <param name="max_vel" value="0.8"/>
  <param name="max_acc" value="0.5"/>
</node>
```

### 3. 安全限制

```xml
<!-- 速度限幅节点（可选） -->
<node pkg="topic_tools" type="throttle" name="cmd_vel_limiter" 
      args="messages /mpc_cmd_vel 10 /hf_platform/twist_mux/cmd_vel"/>

<!-- 或使用速度平滑滤波器 -->
<node pkg="yocs_velocity_smoother" name="velocity_smoother" type="velocity_smoother">
  <rosparam>
    speed_lim_v: 0.8
    speed_lim_w: 1.0
    accel_lim_v: 0.5
    accel_lim_w: 0.8
  </rosparam>
  <remap from="velocity_smoother/raw_cmd_vel" to="/mpc_cmd_vel"/>
  <remap from="velocity_smoother/smooth_cmd_vel" to="/hf_platform/twist_mux/cmd_vel"/>
</node>
```

---

## 完整系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        hf_platform (实物)                        │
├─────────────────────────────────────────────────────────────────┤
│  - SICK 激光雷达 (front + rear) → /merged_cloud                 │
│  - 底盘驱动 → /odom                                              │
│  - AMCL 定位 → /amcl_pose                                        │
│  - TF: map → odom → base_link                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   lidar2world (点云转换)                         │
├─────────────────────────────────────────────────────────────────┤
│  输入: /merged_cloud (sensor_frame)                              │
│  输出: /point_cloud_map (odom frame)                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│              FastPlanner (局部轨迹规划)                          │
├─────────────────────────────────────────────────────────────────┤
│  输入: /point_cloud_map, /odom, /move_base_simple/goal          │
│  输出: /planning/bspline (B样条轨迹)                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                 MPC Tracking (轨迹跟踪)                          │
├─────────────────────────────────────────────────────────────────┤
│  输入: /planning/bspline, /odom                                  │
│  输出: /hf_platform/twist_mux/cmd_vel                            │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   hf_platform 底盘执行                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 与现有 track_simple 对比

| 特性 | track_simple | mpc_track_fastplanner |
|------|-------------|---------------------|
| **轨迹来源** | 手柄控制 / 预定义轨迹 | FastPlanner 实时规划 |
| **控制器** | PID + MPC | 纯 MPC |
| **液体晃动** | ✅ 支持 | ❌ 不支持 |
| **避障** | ❌ 无 | ✅ 实时避障 |
| **适用场景** | 结构化环境，预知轨迹 | 未知环境，动态避障 |
| **计算复杂度** | 中等 | 高 |

**建议**：
- 如果需要**液体晃动控制** + **避障**，可以考虑集成两个系统
- 将 `mpc_track_fastplanner` 的规划模块与 `track_simple` 的 MPC 控制器结合

---

## 下一步工作

### 短期目标

1. ✅ 修改代码适配 hf_platform 话题
2. ✅ 创建实物启动文件
3. ⏳ 在实物平台上测试基本功能
4. ⏳ 调试 TF 变换和点云处理

### 中期目标

1. ⏳ 性能优化（降低计算负载）
2. ⏳ 集成液体晃动约束（从 track_simple 移植）
3. ⏳ 添加安全监控（碰撞检测、紧急停止）

### 长期目标

1. ⏳ 多传感器融合（激光 + 视觉）
2. ⏳ 动态环境适应（移动障碍物）
3. ⏳ 全局路径规划集成

---

## 参考文档

- [FastPlanner 官方文档](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
- [hf_platform 使用手册](../hf_bringup/README.md)
- [track_simple 参数文档](../communication_rs485/docs/PARAMETERS.md)
- [MPC 理论](../communication_rs485/doc/非线性晃动高度项的MPC求解方法.md)

