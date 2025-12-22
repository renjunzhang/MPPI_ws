# FastPlanner 完整数据流详解

## 整体系统架构

```
输入层（感知）
    ↓
地图构建层（SDF Map）
    ↓
规划决策层（FSM + 路径搜索）
    ↓
轨迹优化层（B样条优化）
    ↓
输出层（轨迹发布）
    ↓
跟踪层（MPC控制器）
```

---

## 第一层：输入数据流

### 1.1 订阅的话题

```cpp
// fast_planner_node.cpp - main()
ros::init(argc, argv, "fast_planner_node");
ros::NodeHandle nh("~");

// lidar.xml 的话题重映射
<remap from="/odom_world" to="$(arg odometry_topic)"/>  // 里程计：/odom
<remap from="/sdf_map/cloud" to="$(arg cloud_topic)"/>  // 点云：/point_cloud_map
<remap from="/sdf_map/pose" to="$(arg camera_pose_topic)"/>    // 相机姿态
<remap from="/sdf_map/depth" to="$(arg depth_topic)"/>  // 深度图

// 路点话题
/waypoint_generator/waypoints  → 目标点
```

### 1.2 数据输入流（时间序列）

```
时刻 t=0
├─ 里程计话题 /odom
│  └─ nav_msgs/Odometry
│     ├─ position: (x, y, z)        ← 当前位置
│     ├─ orientation: (qx,qy,qz,qw) ← 当前姿态
│     └─ twist: (vx, vy, vz, wx, wy, wz) ← 当前速度/角速度
│
├─ 点云话题 /point_cloud_map
│  └─ sensor_msgs/PointCloud2       ← 环境障碍物点云
│
└─ 路点话题 /waypoint_generator/waypoints
   └─ nav_msgs/Path
      └─ poses[0]                    ← 目标位置
```

### 1.3 里程计回调（odometryCallback）

```cpp
void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    // 提取位置
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 提取速度
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 提取方向（四元数）
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;  // 标记已接收里程计
}
```

---

## 第二层：地图构建（SDF Map）

### 2.1 地图初始化

```cpp
// FastPlannerManager::initPlanModules()
sdf_map_.reset(new SDFMap);
sdf_map_->initMap(nh);  // 初始化有向距离场地图
```

### 2.2 地图参数（lidar.xml）

```xml
<!-- 地图配置 -->
<param name="sdf_map/resolution" value="0.15"/>              <!-- 栅格分辨率 0.15m -->
<param name="sdf_map/map_size_x" value="40.0"/>              <!-- X方向大小 40m -->
<param name="sdf_map/map_size_y" value="40.0"/>              <!-- Y方向大小 40m -->
<param name="sdf_map/map_size_z" value="0.5"/>               <!-- Z方向大小 0.5m -->
<param name="sdf_map/local_update_range_x" value="5.5"/>     <!-- 局部更新范围 -->
<param name="sdf_map/local_update_range_y" value="5.5"/>
<param name="sdf_map/local_update_range_z" value="4.5"/>
<param name="sdf_map/obstacles_inflation" value="0.3"/>      <!-- 膨胀半径 0.3m -->
```

### 2.3 点云处理流程

```
/point_cloud_map (来自 lidar2world_node 的点云)
    ↓
SDFMap::updateMap()
    ├─ 体素化：将点云离散化成 0.15m 网格
    ├─ 标记占用：标记有障碍物的网格
    ├─ 膨胀：将占用网格膨胀 0.3m（安全距离）
    └─ 计算 SDF：使用距离变换计算每个网格到障碍物的距离
    ↓
Distance Field（有向距离场）
└─ dist(x,y,z) = 该点到最近障碍物的距离
   如果 dist > 0.3 → 安全
   如果 dist < 0 → 碰撞区域
```

### 2.4 地图示例

```
俯视图（XY平面）：

      Obstacle (点云)
           ▓▓▓
          ▓▓▓▓▓
     膨胀↓
         ░░░░░
        ░▓▓▓▓▓░  (障碍物+0.3m膨胀)
       ░░▓▓▓▓▓░░

Start(x,y)      Goal(x,y)
  S ─────────────→ G

SDF 值：
- S处：dist = 2.5m（距障碍物2.5m远）
- G处：dist = 1.2m（距障碍物1.2m远）
```

---

## 第三层：FSM 状态机

### 3.1 状态定义

```cpp
enum FSM_EXEC_STATE {
    INIT = 0,           // 初始化状态
    WAIT_TARGET = 1,    // 等待目标点
    GEN_NEW_TRAJ = 2,   // 生成新轨迹
    EXEC_TRAJ = 3,      // 执行轨迹
    REPLAN_TRAJ = 4     // 重新规划
};
```

### 3.2 完整状态流转图

```
┌─────────────┐
│   INIT      │  启动时，等待里程计和目标点
│ have_odom_? │
└──────┬──────┘
       │ YES (收到里程计数据)
       ↓
┌─────────────────┐
│  WAIT_TARGET    │  等待用户设置目标点
│ have_target_?   │  (通过 /waypoint_generator/waypoints)
└──────┬──────────┘
       │ YES (收到目标点)
       ↓
┌────────────────────────┐
│   GEN_NEW_TRAJ         │
│                        │
│ 1. 获取当前状态：      │
│    start_pt_  ← 位置   │
│    start_vel_ ← 速度   │
│    start_yaw_ ← 方向   │
│                        │
│ 2. 调用规划器：        │
│    kinodynamicReplan() │
│    ├─ A* 路径搜索      │
│    ├─ 轨迹优化         │
│    └─ 方向规划         │
│                        │
│ 3. 发布 B样条轨迹      │
│    → /planning/bspline │
└──────┬─────────────────┘
       │ success?
       ├─ YES ↓
       │    ┌───────────────────┐
       │    │   EXEC_TRAJ       │
       │    │                   │
       │    │ 执行当前轨迹，    │
       │    │ 每 10ms 检查：    │
       │    │ ① 到达目标？     │
       │    │    → WAIT_TARGET │
       │    │ ② 碰撞检测？     │
       │    │    → REPLAN_TRAJ │
       │    │ ③ 距离起点<1.5m？ │
       │    │    → REPLAN_TRAJ │
       │    └───────┬───────────┘
       │            │ 需要重规划
       │            ↓
       │    ┌───────────────────┐
       │    │  REPLAN_TRAJ      │
       │    │                   │
       │    │ 从当前执行位置    │
       │    │ 重新开始规划      │
       │    │ (类似GEN_NEW_TRAJ)│
       │    └───────┬───────────┘
       │            │
       │            └─→ 返回 EXEC_TRAJ (循环)
       │
       └─ NO (规划失败)
          重试规划

```

### 3.3 FSM 回调（100Hz 执行）

```cpp
void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
    // 定时器：100Hz (0.01s 调用一次)
    
    switch (exec_state_) {
        case INIT:
            // 检查是否收到里程计和目标点
            // 条件满足 → WAIT_TARGET
            
        case WAIT_TARGET:
            // 等待目标点
            // 目标点到达 → GEN_NEW_TRAJ
            
        case GEN_NEW_TRAJ:
            // 调用规划器生成轨迹
            callKinodynamicReplan();
            // 成功 → EXEC_TRAJ，失败重试
            
        case EXEC_TRAJ:
            // 检查是否需要重规划
            // ① 到达目标 → WAIT_TARGET
            // ② 碰撞 → REPLAN_TRAJ
            // ③ 距起点太近 → REPLAN_TRAJ
            
        case REPLAN_TRAJ:
            // 从当前位置重新规划
            callKinodynamicReplan();
            // 成功 → EXEC_TRAJ
    }
}
```

---

## 第四层：核心规划算法

### 4.1 规划流程（callKinodynamicReplan）

```cpp
bool KinoReplanFSM::callKinodynamicReplan() {
    
    // ===== 第1步：运动学路径搜索 =====
    bool plan_success = planner_manager_->kinodynamicReplan(
        start_pt_,      // 起点位置
        start_vel_,     // 起点速度
        start_acc_,     // 起点加速度
        end_pt_,        // 目标位置
        end_vel_        // 目标速度(通常为0)
    );
    
    if (!plan_success) return false;
    
    // ===== 第2步：方向规划 =====
    planner_manager_->planYaw(start_yaw_);
    
    // ===== 第3步：发布 B样条轨迹 =====
    auto info = &planner_manager_->local_data_;
    
    plan_manage::Bspline bspline;
    bspline.order = 3;  // 3阶B样条
    bspline.start_time = info->start_time_;
    
    // 获取位置控制点
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
    }
    
    // 获取节点向量
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
    }
    
    // 获取方向控制点
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
        bspline.yaw_pts.push_back(yaw_pts(i, 0));
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();
    
    // 发布轨迹
    bspline_pub_.publish(bspline);  → /planning/bspline
    
    return true;
}
```

### 4.2 运动学搜索（Kino_Replan）详解

```
Step 1: 初始化搜索空间
├─ 栅格分辨率（spatial）: 0.1m
├─ 时间分辨率（temporal）: 0.8s
└─ 最大时间间隔（max_tau）: 1.0s

Step 2: 构建状态节点
start_state = {x, y, z, vx, vy, vz, ax, ay, az}
            = {start_pt_, start_vel_, start_acc_}

Step 3: A* 搜索（时间+空间 3D 搜索）
for each node in open_set:
    for each time_step in [0, max_tau]:
        for each acc_control in [-max_acc, +max_acc]:
            // 计算下一时刻状态
            next_pos = pos + vel*dt + 0.5*acc*dt²
            next_vel = vel + acc*dt
            
            // 检查碰撞
            if checkCollision(next_pos):
                continue  // 跳过碰撞状态
            
            // 计算启发值（到目标的距离）
            h_value = distance(next_pos, end_pt_)
            
            // 加入搜索队列
            open_set.push({next_pos, g_value + cost, h_value})

Step 4: 回溯最优路径
path = backtrack(goal_node)  // 从目标节点反向追溯到起点
```

### 4.3 轨迹优化（B样条优化）

```
Input: 粗路径（离散点序列）
       
Step 1: B样条参数化
├─ 将路径转换为 3阶 B样条
├─ 提取控制点
└─ 建立节点向量

Step 2: 优化目标函数
minimize:
  J = λ₁·(平滑项) + λ₂·(合理性项) + λ₃·(时间项) + λ₄·(方向变化项)
  
  其中：
  - 平滑项：∫ κ² ds （曲率平方和）
  - 合理性项：∫ (v²/a_max) ds （加速度约束）
  - 时间项：总耗时
  - 方向项：∑ Δyaw² （避免方向突变）

Step 3: 约束条件
├─ 速度约束：v ≤ v_max
├─ 加速度约束：a ≤ a_max
├─ 碰撞约束：dist_to_obstacle ≥ 0.3m
└─ 初末端条件：过起点和目标点

Step 4: 求解优化
使用梯度下降法（如 L-BFGS）迭代优化轨迹
↓
Output: 平滑、可行的 B样条轨迹
```

### 4.4 方向规划（planYaw）

```
输入：B样条轨迹的控制点

目标：规划方向使得小车始终指向前进方向

算法：
1. 计算轨迹的切向量（导数）
   tangent = trajectory.derivative()
   
2. 从每个切向量计算偏航角
   yaw = atan2(tangent.y, tangent.x)
   
3. 处理角度连续性（消除 2π 跳变）
   if (yaw[i] - yaw[i-1] > π):
       yaw[i] -= 2π
   
4. 平滑方向序列
   使用样条插值光滑处理方向

输出：方向 B样条轨迹（yaw_traj_）
```

---

## 第五层：B样条轨迹发布

### 5.1 B样条消息格式（mpc_tracking::Bspline）

```cpp
struct Bspline {
    int32 order;                          // 样条阶数（通常为3）
    ros::Time start_time;                 // 轨迹开始时间
    int32 traj_id;                        // 轨迹ID
    
    geometry_msgs/Point[] pos_pts;        // 位置控制点
    float64[] knots;                      // 节点向量
    
    float64[] yaw_pts;                    // 方向控制点
    float64 yaw_dt;                       // 方向B样条的参数间隔
};
```

### 5.2 轨迹发布

```cpp
bspline_pub_.advertise<plan_manage::Bspline>("/planning/bspline", 10);

// 规划完成时发布
bspline_pub_.publish(bspline_msg);  → 发送到 /planning/bspline 话题
```

### 5.3 轨迹示例（二维平面）

```
XY平面俯视图：

目标点 G
  ▲
  │
  │ ← B样条曲线（红色）
  │╱╲╱╲╱╲╱╲
  │
  │
  └──────────→ Start S

B样条特性：
- 通过起点和终点
- 避开所有障碍物（膨胀0.3m）
- 曲率平滑连续
- 满足速度/加速度约束
```

---

## 第六层：重规划触发条件

### 6.1 重规划判断（EXEC_TRAJ 状态）

```cpp
case EXEC_TRAJ: {
    LocalTrajData* info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    
    Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
    
    // ===== 条件1：轨迹已执行完毕 =====
    if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
    }
    
    // ===== 条件2：接近目标点（不需要重规划） =====
    else if ((end_pt_ - pos).norm() < no_replan_thresh_) {  // 距离 < 1.0m
        return;  // 继续执行，不重规划
    }
    
    // ===== 条件3：太接近起点（不需要重规划） =====
    else if ((info->start_pos_ - pos).norm() < replan_thresh_) {  // 距离 > 1.5m
        return;  // 继续执行，不重规划
    }
    
    // ===== 条件4：需要重规划 =====
    else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
    }
}
```

### 6.2 碰撞检测（20Hz 执行，checkCollisionCallback）

```cpp
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
    // 定时器：20Hz (0.05s 调用一次)
    
    if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
        double dist;
        bool safe = planner_manager_->checkTrajCollision(dist);
        
        if (!safe) {
            ROS_WARN("current traj in collision.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            
            // 发布重规划信号
            std_msgs::Empty msg;
            replan_pub_.publish(msg);
        }
    }
}
```

---

## 第七层：数据流完整时序图

```
时刻 t₀: 启动
├─ fast_planner_node 启动
├─ KinoReplanFSM 初始化
└─ 订阅话题（等待数据）

时刻 t₁: 第一条里程计消息到达
├─ odometryCallback()
├─ odom_pos_ ← (x, y, z)
├─ odom_vel_ ← (vx, vy, vz)
├─ have_odom_ = true
└─ FSM: INIT → WAIT_TARGET

时刻 t₂: 第一条点云消息到达
├─ sdf_map_->updateMap()
├─ 构建 SDF 距离场
└─ 准备就绪

时刻 t₃: 用户设置目标点
├─ waypointCallback()
├─ end_pt_ ← 目标位置
├─ have_target_ = true
└─ FSM: WAIT_TARGET → GEN_NEW_TRAJ

时刻 t₄: 执行规划（100Hz）
├─ execFSMCallback()
├─ GEN_NEW_TRAJ:
│  ├─ kinodynamicReplan()
│  │  ├─ A* 搜索（0-200ms）
│  │  ├─ B样条优化（0-50ms）
│  │  └─ planYaw()
│  ├─ 发布 /planning/bspline
│  └─ FSM: GEN_NEW_TRAJ → EXEC_TRAJ

时刻 t₅: 执行轨迹（连续）
├─ FSM: EXEC_TRAJ
├─ 每 10ms 检查重规划条件
├─ 每 50ms 检查碰撞
├─ 如果需要重规划：
│  └─ FSM: EXEC_TRAJ → REPLAN_TRAJ → GEN_NEW_TRAJ
└─ 持续发布 /planning/bspline

时刻 t₆: 到达目标
├─ t_cur > duration
├─ FSM: EXEC_TRAJ → WAIT_TARGET
└─ 等待新目标点
```

---

## 第八层：输出话题总结

### 8.1 FastPlanner 发布的话题

```
/planning/bspline
├─ 类型：plan_manage::Bspline
├─ 频率：规划时发布（通常 0.5-2Hz）
├─ 内容：
│  ├─ 位置B样条（控制点+节点向量）
│  ├─ 方向B样条（yaw 控制点）
│  └─ 轨迹时间戳和ID
└─ 用途：供 MPC 跟踪控制器使用

/planning/replan
├─ 类型：std_msgs::Empty
├─ 频率：重规划时发布
├─ 用途：信号量，表示"重规划触发"

/planning/new
├─ 类型：std_msgs::Empty
├─ 频率：新轨迹生成时发布
└─ 用途：信号量，表示"新轨迹可用"

可视化话题（RViz）：
├─ /planning/geometric_path     # A*搜索的几何路径
├─ /planning/bspline            # B样条轨迹
└─ /planning/goal               # 目标点
```

### 8.2 数据流到 MPC 跟踪器

```
/planning/bspline
    ↓
mpc_tracking_node
    ├─ 订阅并解析 B样条轨迹
    ├─ 提取 N=30 步（3秒）的期望状态
    ├─ 调用 MPC 求解器
    ├─ 得到最优速度命令 (u, v, r)
    └─ 发布 /cmd_vel
        ↓
    omni_fake_node 或实物底盘驱动
        ↓
    小车执行并反馈 /odom
        ↓
    FastPlanner 继续接收 /odom 用于下一步规划
        ↓
    形成闭环
```

---

## 完整数据流图（俯视图）

```
┌──────────────┐
│ Gazebo/实物  │ 发送传感器数据
└───────┬──────┘
        │ /velodyne_points, /odom, /imu/data
        ↓
┌──────────────────────────┐
│ lidar2world_node         │ 坐标系转换
│ 将点云从 base_link → odom│
└───────┬──────────────────┘
        │ /point_cloud_map
        ↓
┌──────────────────────────┐
│ fast_planner_node        │ 核心规划模块
│ ├─ SDFMap (地图构建)     │
│ ├─ FSM (状态机)          │
│ ├─ KinodynamicAstar      │
│ ├─ BsplineOptimizer      │
│ └─ Visualization         │
└───────┬──────────────────┘
        │ /planning/bspline
        ↓
┌──────────────────────────┐
│ mpc_tracking_node        │ 轨迹跟踪
│ ├─ B样条解析             │
│ ├─ MPC 求解器 (Ipopt)   │
│ └─ 控制输出              │
└───────┬──────────────────┘
        │ /cmd_vel
        ↓
┌──────────────────────────┐
│ 小车执行运动             │
│ (omni_fake/实物底盘)     │
└───────┬──────────────────┘
        │ /odom (反馈)
        └──────────→ 回到 FastPlanner（闭环）
```

---

## 关键数据结构

### 8.1 LocalTrajData（本地轨迹数据）

```cpp
struct LocalTrajData {
    // 时间信息
    ros::Time start_time_;
    double duration_;  // 轨迹持续时间
    int traj_id_;
    
    // 轨迹
    NonUniformBspline position_traj_;   // 位置B样条
    NonUniformBspline velocity_traj_;   // 速度B样条
    NonUniformBspline acceleration_traj_; // 加速度B样条
    NonUniformBspline yaw_traj_;        // 方向B样条
    NonUniformBspline yawdot_traj_;     // 角速度B样条
    NonUniformBspline yawdotdot_traj_;  // 角加速度B样条
    
    // 起点信息
    Eigen::Vector3d start_pos_;
    Eigen::Vector3d start_vel_;
};
```

### 8.2 NonUniformBspline 类

```cpp
class NonUniformBspline {
    // 核心方法
    Eigen::Vector3d evaluateDeBoorT(double t);  // 计算参数t处的位置
    Eigen::Vector3d evaluateDerivative(double t); // 计算导数（速度）
    NonUniformBspline getDerivative();  // 获得导数B样条对象
    
    // 参数
    Eigen::MatrixXd control_points_;    // 控制点矩阵
    Eigen::VectorXd knots_;             // 节点向量
    int order_;                         // 阶数（通常为3）
};
```

---

## 性能指标（从代码中提取）

```
规划频率：
- FSM 循环：100 Hz (0.01s)
- 碰撞检测：20 Hz (0.05s)
- MPC 控制：10 Hz (0.1s)

规划耗时（参数）：
- A* 搜索最大耗时：0.5s (max_cpu_time)
- 轨迹优化最大迭代：300次 (max_iteration_num2)
- 单次优化最大时间：0.005s

约束参数：
- 最大速度：3 m/s
- 最大加速度：2 m/s²
- 最大角速度：1 rad/s
- 碰撞检测边界：0.2m + 0.3m膨胀 = 0.5m

地图参数：
- 地图范围：40×40×0.5 m
- 分辨率：0.15 m
- 局部更新范围：5.5×5.5×4.5 m
- 膨胀距离：0.3 m
```

---

## 总结

**FastPlanner 的核心数据流：**

```
原始感知数据
(点云+里程计+目标点)
    ↓
SDF 地图构建
(栅格+距离场)
    ↓
FSM 状态决策
(何时规划/重规划)
    ↓
Kino_Replan 规划
(A*搜索+约束处理)
    ↓
B样条优化
(平滑+可行性)
    ↓
方向规划
(偏航角连续性)
    ↓
轨迹发布
(/planning/bspline)
    ↓
MPC 跟踪控制
(小车执行)
    ↓
反馈到下一规划周期
(闭环系统)
```
