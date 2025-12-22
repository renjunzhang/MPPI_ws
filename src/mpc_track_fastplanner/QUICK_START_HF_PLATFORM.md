# hf_platform 实物运行快速指南

## ✅ 代码修改完成

以下文件已完成适配：

1. **`mpc_node.cpp`** - MPC 控制器
   - ✅ cmd_vel 话题：`/cmd_vel` → `/hf_platform/twist_mux/cmd_vel`
   - ✅ 支持参数化配置
   - ✅ 添加启动信息输出

2. **`transform.cpp`** - 点云转换
   - ✅ 输入话题：`/velodyne_points` → `/merged_cloud`
   - ✅ 自动检测源坐标系（`scan_merge`）
   - ✅ 支持参数化配置
   - ✅ 添加限流日志

3. **`hf_platform.launch`** - 实物启动文件
   - ✅ 完整集成所有模块
   - ✅ 适配 hf_platform 话题
   - ✅ 安全速度限制

---

## 🚀 快速启动（推荐）

### 方式 1：使用智能检查脚本（最推荐）

```bash
# 终端 1：启动 hf_platform
roslaunch junjun_bringup junjun_bringup.launch

# 终端 2：运行系统检查并启动
/home/a/hf_move/src/mpc_track_fastplanner/launch/check_and_start.sh
# 脚本会自动检查系统状态，并提供三种模式：
#   1. 正常模式（控制底盘）
#   2. 调试模式（不控制底盘，安全测试）
#   3. 仅检查，不启动
```

### 方式 2：直接启动（跳过检查）

```bash
# 终端 1：启动 hf_platform
roslaunch junjun_bringup junjun_bringup.launch

# 终端 2：正常模式（控制底盘）
roslaunch mpc_track_fastplanner hf_platform.launch

# 或调试模式（不控制底盘）
roslaunch mpc_track_fastplanner hf_platform_debug.launch
```

### 方式 3：使用旧的启动脚本

```bash
# 终端 1：启动 hf_platform
roslaunch junjun_bringup junjun_bringup.launch

# 终端 2：启动 mpc_track_fastplanner
/home/a/hf_move/src/mpc_track_fastplanner/launch/start_hf_platform.sh
```

---

## 📊 系统架构

```
junjun_bringup (hf_platform)
  ├─ /odom (20Hz)                     ← 里程计
  ├─ /merged_cloud (34Hz)             ← 激光点云 (scan_merge)
  └─ /hf_platform/twist_mux/cmd_vel   ← 速度命令接收
           ↓
    lidar2world_node
  输入: /merged_cloud (scan_merge frame)
  输出: /point_cloud_map (odom frame)
           ↓
    FastPlanner (plan_manage)
  输入: /point_cloud_map, /odom, /move_base_simple/goal
  输出: /planning/bspline (B样条轨迹)
           ↓
    mpc_tracking_node
  输入: /planning/bspline, /odom
  输出: /hf_platform/twist_mux/cmd_vel
           ↓
    hf_platform 底盘执行
```

---

## 🎯 使用步骤

### 推荐：三步冒烟测试法（基于 GPT 建议）

#### 第一步：感知与规划测试（不动底盘）

```bash
# 1. 启动底层
roslaunch junjun_bringup junjun_bringup.launch

# 2. 启动调试模式
roslaunch mpc_track_fastplanner hf_platform_debug.launch

# 3. 在 RViz 中检查：
#    - Fixed Frame 设为 odom
#    - 查看 /point_cloud_map（点云是否正常）
#    - 点击 2D Nav Goal 发送目标
#    - 查看 /planning/bspline（是否生成轨迹）
```

**判断标准**：
- ✅ 点云能在 RViz 中正常显示
- ✅ 发送目标后能看到规划轨迹
- ✅ 无 TF 变换错误

#### 第二步：MPC 跟踪测试（空转）

```bash
# 在调试模式下继续
# 在 RViz 中添加：
#   - Path → /mpc_predict_path（MPC 预测）
#   - Path → /mpc_motion_path（实际运动）

# 监控 MPC 输出（调试话题）
rostopic echo /mpc/debug_cmd_vel
```

**判断标准**：
- ✅ `/mpc_predict_path` 跟随参考轨迹
- ✅ 速度命令无抖动、数值合理
- ✅ 没有求解器报错

#### 第三步：底盘实际运行

```bash
# 关闭调试模式，启动正常模式
roslaunch mpc_track_fastplanner hf_platform.launch

# 监控 twist_mux 选择
rostopic echo /hf_platform/twist_mux/selected

# 确认：selected 应显示 MPC 通道
```

**判断标准**：
- ✅ twist_mux 选择了 MPC 通道
- ✅ 机器人按轨迹运动
- ✅ 无碰撞、无抖动

---

### 传统步骤（快速上手）

### Step 1: 启动底层系统

```bash
roslaunch junjun_bringup junjun_bringup.launch
```

**确认启动成功**：
```bash
# 检查关键话题
rostopic hz /odom              # 应该是 20Hz
rostopic hz /merged_cloud      # 应该是 34Hz

# 检查 TF 树
rosrun tf tf_echo odom hf_base_link
```

### Step 2: 启动 FastPlanner + MPC

```bash
roslaunch mpc_track_fastplanner hf_platform.launch
```

**启动后应该看到**：
```
=================================================
[lidar2world] Node started with configuration:
  input_cloud: /merged_cloud
  output_cloud: /point_cloud_map
  target_frame: odom
=================================================

=================================================
[MPC Tracking] Node started with configuration:
  cmd_vel_topic: /hf_platform/twist_mux/cmd_vel
  odom_topic: /odom
=================================================
```

### Step 3: 在 RViz 中操作

1. **确认 Fixed Frame** 设置为 `odom`
2. **添加显示**：
   - `PointCloud2` → `/merged_cloud` (原始激光)
   - `PointCloud2` → `/point_cloud_map` (转换后的点云)
   - `Path` → `/mpc_predict_path` (MPC 预测)
   - `Path` → `/mpc_motion_path` (实际轨迹)
3. **发送目标**：
   - 点击工具栏 "2D Nav Goal"
   - 在地图上点击目标位置和朝向
   - 系统自动规划并执行

---

## ⚙️ 参数调整

### 修改速度限制

编辑 `/home/a/hf_move/src/mpc_track_fastplanner/launch/hf_platform.launch`：

```xml
<!-- 速度限制（实物安全值） -->
<arg name="max_vel" default="0.5"/>  <!-- 改为 0.3-0.8 m/s -->
<arg name="max_acc" default="0.4"/>  <!-- 改为 0.2-0.6 m/s² -->
```

### 修改地图范围

```xml
<!-- FastPlanner 地图参数 -->
<arg name="map_size_x" default="20.0"/>  <!-- 改为实际场地大小 -->
<arg name="map_size_y" default="20.0"/>
```

### 运行时参数调整

```bash
# 动态调整速度（需要 FastPlanner 支持）
rosparam set /fast_planner/max_vel 0.3

# 查看当前参数
rosparam get /mpc_tracking_node/cmd_vel_topic
```

---

## 🔍 监控与调试

### 实时监控

```bash
# 终端 3：监控速度命令
rostopic echo /hf_platform/twist_mux/cmd_vel

# 终端 4：监控点云转换
rostopic hz /point_cloud_map

# 终端 5：监控 FastPlanner 状态
rostopic echo /planning/bspline
```

### 查看话题列表

```bash
rostopic list | grep -E "(mpc|planning|point_cloud)"
```

**应该看到**：
- `/point_cloud_map` - 转换后的点云
- `/planning/bspline` - FastPlanner 轨迹
- `/planning/pos_cmd` - 位置命令
- `/mpc_predict_path` - MPC 预测路径
- `/mpc_motion_path` - 实际运动路径

---

## ⚠️ 常见问题

### 问题 1: 点云不显示

**症状**：RViz 中看不到 `/point_cloud_map`

**解决**：
```bash
# 检查输入点云
rostopic hz /merged_cloud

# 检查 TF 变换
rosrun tf tf_echo odom scan_merge

# 查看 lidar2world 日志
rosnode info /lidar2world_node
```

### 问题 2: 机器人不动

**症状**：规划成功但机器人不执行

**解决**：
```bash
# 检查速度命令
rostopic echo /hf_platform/twist_mux/cmd_vel

# 检查 twist_mux 优先级
rostopic echo /hf_platform/twist_mux/selected

# 确认没有其他控制源占用
rostopic info /hf_platform/twist_mux/cmd_vel
```

### 问题 3: FastPlanner 规划失败

**症状**：点击目标点后无轨迹生成

**解决**：
```bash
# 检查目标点是否在地图范围内
# 在 RViz 中查看 /point_cloud_map 的范围

# 降低速度要求
rosparam set /fast_planner/max_vel 0.3

# 查看 FastPlanner 日志
# 在启动的终端中查看错误信息
```

### 问题 4: TF 变换错误

**症状**：`Failed to transform point cloud`

**解决**：
```bash
# 查看 TF 树
rosrun tf view_frames
evince frames.pdf

# 检查关键变换
rosrun tf tf_echo odom hf_base_link
rosrun tf tf_echo odom scan_merge

# 确认 TF 发布频率
rostopic hz /tf
```

---

## 🔧 性能优化

### 降低计算负载

如果系统运行卡顿，可以：

1. **降低点云分辨率**：编辑 `transform.cpp` 添加下采样
2. **降低规划频率**：修改 FastPlanner 配置
3. **减少预测地平线**：修改 MPC 参数

### 提高响应速度

1. **增加控制频率**：修改 `mpc_node.cpp` 的 `Duration(0.1)` → `Duration(0.05)`
2. **优化路径**：调整 FastPlanner 的平滑参数
3. **减小地图范围**：降低 `map_size_x/y`

---

## 📝 系统检查清单

启动前确认：

- [ ] `roslaunch junjun_bringup` 已运行
- [ ] `/odom` 话题正常（20Hz）
- [ ] `/merged_cloud` 话题正常（34Hz）
- [ ] TF 树完整（odom → hf_base_link → scan_merge）

启动后确认：

- [ ] `/point_cloud_map` 正常发布
- [ ] RViz 能看到点云和机器人模型
- [ ] 能用 2D Nav Goal 发送目标
- [ ] `/planning/bspline` 有轨迹输出
- [ ] `/hf_platform/twist_mux/cmd_vel` 有速度命令

---

## 📚 相关文件

- **代码文件**：
  - `/home/a/hf_move/src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp`
  - `/home/a/hf_move/src/mpc_track_fastplanner/src/lidar2world/src/transform.cpp`

- **启动文件**：
  - `/home/a/hf_move/src/mpc_track_fastplanner/launch/hf_platform.launch`
  - `/home/a/hf_move/src/mpc_track_fastplanner/launch/start_hf_platform.sh`

- **文档**：
  - `/home/a/hf_move/src/mpc_track_fastplanner/PLATFORM_MIGRATION_GUIDE.md` - 完整迁移指南
  - `/home/a/hf_move/src/mpc_track_fastplanner/README.md` - 原始项目说明

---

## 🎉 测试建议

### 初次测试

1. **选择近距离目标**（2-3米）
2. **空旷环境测试**（减少障碍物）
3. **低速运行**（max_vel=0.3）
4. **观察轨迹平滑性**

### 进阶测试

1. **增加障碍物**（测试避障）
2. **提高速度**（max_vel=0.5-0.8）
3. **远距离目标**（5-10米）
4. **复杂环境**（多障碍物）

---

## 🆘 紧急停止

如果需要紧急停止：

```bash
# 方法 1：Ctrl+C 停止 launch 文件

# 方法 2：发送零速度
rostopic pub -1 /hf_platform/twist_mux/cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# 方法 3：关闭 mpc_tracking 节点
rosnode kill /mpc_tracking_node
```

---

## 📞 获取帮助

如有问题，请查看：

1. 本文档的"常见问题"部分
2. `PLATFORM_MIGRATION_GUIDE.md` 完整迁移指南
3. FastPlanner 官方文档：https://github.com/HKUST-Aerial-Robotics/Fast-Planner

祝使用愉快！🎊
