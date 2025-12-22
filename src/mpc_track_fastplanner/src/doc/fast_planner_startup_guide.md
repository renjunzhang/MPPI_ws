# FastPlanner 启动流程分析

## 启动命令
```bash
roslaunch plan_manage 16_lidar.launch
```

## 启动流程

### 1. **Launch 文件加载**
```
16_lidar.launch
  ├─ 设置参数
  │  ├─ map_size_x/y/z (地图大小)
  │  ├─ odom_topic = "/odom"
  │  ├─ max_vel = 3 m/s
  │  └─ max_acc = 2 m/s²
  │
  └─ 包含 lidar.xml
      ├─ 启动 fast_planner_node
      ├─ 启动 traj_server
      └─ 启动 waypoint_generator
```

### 2. **主程序入口**（fast_planner_node.cpp）

**文件位置：** `/src/Fast-Planner/fast_planner/plan_manage/src/fast_planner_node.cpp`

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);  // 读取参数: 1=Kino, 2=Topo

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;

  if (planner == 1) {
    kino_replan.init(nh);  ← 初始化 Kino_Replan
  } else if (planner == 2) {
    topo_replan.init(nh);  ← 初始化 Topo_Replan
  }

  ros::Duration(1.0).sleep();
  ros::spin();  ← 进入 ROS 循环
  return 0;
}
```

### 3. **KinoReplanFSM 初始化**（kino_replan_fsm.cpp）

**文件位置：** `/src/Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp`

```cpp
void KinoReplanFSM::init(ros::NodeHandle& nh) {
  // 初始化状态变量
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_ = false;

  // 从参数服务器读取 FSM 参数
  nh.param("fsm/flight_type", target_type_, -1);      // 飞行类型: 1=手动, 2=预设路点
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  
  // 读取所有路点坐标
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  // 初始化核心模块
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);        // 初始化地图、搜索、优化器
  visualization_.reset(new PlanningVisualization(nh));

  // 创建定时器（执行 FSM 状态机）
  exec_timer_ = nh.createTimer(
    ros::Duration(0.01),                         // 100 Hz
    &KinoReplanFSM::execFSMCallback, this
  );
  safety_timer_ = nh.createTimer(
    ros::Duration(0.05),                         // 20 Hz
    &KinoReplanFSM::checkCollisionCallback, this
  );

  // 订阅输入话题
  waypoint_sub_ = nh.subscribe(
    "/waypoint_generator/waypoints", 1, 
    &KinoReplanFSM::waypointCallback, this
  );
  odom_sub_ = nh.subscribe(
    "/odom_world", 1, 
    &KinoReplanFSM::odometryCallback, this
  );

  // 发布输出话题
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
}
```

### 4. **FSM 状态机**

状态流转图：
```
INIT
  ↓
WAIT_TARGET ← 等待目标点
  ↓ (收到 waypoint)
GEN_NEW_TRAJ ← 生成新轨迹
  ↓
EXEC_TRAJ ← 执行轨迹
  ↓ (距离目标 < thresh_replan 或碰撞检测触发)
REPLAN_TRAJ ← 重新规划
  ↓ (新轨迹生成完毕)
EXEC_TRAJ (循环)
```

### 5. **FastPlannerManager 初始化流程**

```cpp
void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  // 初始化 SDF 地图
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  // 初始化路径搜索（A* 搜索器）
  path_search_.reset(new KinodynamicAstar);
  path_search_->setParam(nh);

  // 初始化轨迹优化器（B 样条优化）
  bspline_optimizer_.reset(new BsplineOptimizer);
  bspline_optimizer_->setParam(nh);

  // 初始化 B 样条轨迹生成器
  bspline_.reset(new NonUniformBspline);
}
```

### 6. **话题消息流**

```
输入话题:
  /odom_world          ← 里程计（位置、速度）
  /waypoint_generator/waypoints → 目标路点
  /sdf_map/cloud       ← 点云（障碍物信息）

处理流程:
  waypointCallback() → 触发规划
    ↓
  execFSMCallback() → 执行 FSM（每 10ms）
    ├─ GEN_NEW_TRAJ: 调用规划器生成初始轨迹
    ├─ REPLAN_TRAJ: 动态重新规划
    └─ EXEC_TRAJ: 执行中，监测重规划条件
    ↓

输出话题:
  /planning/bspline    → B样条轨迹（给MPC跟踪器）
  /planning/replan     → 重规划触发信号
  /planning/new        → 新轨迹生成完毕信号
```

### 7. **核心规划流程**（伪代码）

```cpp
void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  switch(exec_state_) {
    case WAIT_TARGET:
      // 等待目标点
      break;
      
    case GEN_NEW_TRAJ:
      // 调用 Kino_Replan 规划器
      planner_manager_->kinodynamicSearch(start, end);  // 运动学搜索
      planner_manager_->bsplineOptimization();           // 轨迹优化
      // 发布 B 样条轨迹到 /planning/bspline
      bspline_pub_.publish(bspline_msg);
      changeFSMExecState(EXEC_TRAJ, "FSM");
      break;
      
    case EXEC_TRAJ:
      // 检查是否需要重规划
      if (dist_to_goal < replan_thresh_) {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
      
    case REPLAN_TRAJ:
      // 同样的规划过程
      planner_manager_->kinodynamicSearch(...);
      planner_manager_->bsplineOptimization();
      bspline_pub_.publish(bspline_msg);
      changeFSMExecState(EXEC_TRAJ, "FSM");
      break;
  }
}
```

## 参数配置位置

**配置文件：** `lidar.xml` 第 36-44 行

```xml
<!-- planner manager -->
<param name="manager/use_geometric_path" value="false" type="bool"/>
<param name="manager/use_kinodynamic_path" value="true" type="bool"/>  ← 启用 Kino_Replan
<param name="manager/use_topo_path" value="false" type="bool"/>
<param name="manager/use_optimization" value="true" type="bool"/>
```

**FastPlanner 配置参数：** `lidar.xml` 第 39-40 行

```xml
<param name="planner_node/planner" value="1" type="int"/>  ← 1=Kino, 2=Topo
<param name="fsm/flight_type" value="1" type="int"/>       ← 飞行类型
```

## 关键代码文件

| 文件 | 位置 | 功能 |
|------|------|------|
| fast_planner_node.cpp | plan_manage/src/ | 主程序入口 |
| kino_replan_fsm.cpp | plan_manage/src/ | FSM 状态机实现 |
| FastPlannerManager | plan_manage/ | 规划模块管理器 |
| KinodynamicAstar | path_searching/ | 运动学搜索算法 |
| BsplineOptimizer | bspline_opt/ | 轨迹优化模块 |
| SDFMap | plan_env/ | 地图表示模块 |

## 数据流总结

```
16_lidar.launch
  ↓
fast_planner_node
  ↓
KinoReplanFSM::init()
  ├─ 初始化 FastPlannerManager（地图、搜索、优化器）
  ├─ 创建定时器（100Hz FSM 循环）
  ├─ 订阅 /odom_world, /waypoint_generator/waypoints
  └─ 创建发布器 /planning/bspline, /planning/replan
  ↓
等待目标点 → 触发规划 → 执行规划 → 发布轨迹
  ↓
mpc_tracking_node 订阅 /planning/bspline
  ↓
MPC 跟踪轨迹 → 输出 /cmd_vel
```
