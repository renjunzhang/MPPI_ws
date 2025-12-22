# 🚀 从这里开始：FastPlanner + MPC 集成到 hf_platform

**你在这里** 👈  
快速入门指南 - 5分钟了解全貌

---

## 一句话说明

将仿真的 **FastPlanner 轨迹规划** + **MPC 跟踪控制** 系统集成到 hf_platform 实物，只需修改两个话题映射。

---

## 集成流程（5分钟）

### 1️⃣ 修改代码（2分钟）

编辑文件：`src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp`

**只需改这两行**（搜索并替换）：

```
行 207：/cmd_vel          →  /hf_platform/joy_vel
行 209：/odom             →  /hf_platform/odom
```

### 2️⃣ 编译（2分钟）

```bash
cd /home/a/hf_move
catkin_make --only mpc_tracking
```

### 3️⃣ 运行（1分钟）

```bash
# 3 个终端，按顺序启动
roslaunch hf_bringup bringup.launch          # 终端 1
roslaunch plan_manage kino.launch            # 终端 2
rosrun mpc_tracking mpc_tracking_node        # 终端 3
```

**完成！** 在 RViz 中点击 Goal → hf_platform 自主运动 ✅

---

## 文档导航

根据你的角色选择相应文档：

### 👨‍💼 项目经理
- **开始**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 5分钟快速参考
- **详看**: [INTEGRATION_CHECKLIST.md](./INTEGRATION_CHECKLIST.md) - 完整集成检查清单

### 👨‍💻 系统集成工程师
- **开始**: [INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md) - 详细集成指南
- **对照**: [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) - 代码修改详解

### 🔧 运维/测试人员
- **开始**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 快速参考卡
- **排查**: [QUICK_REFERENCE.md#快速诊断](./QUICK_REFERENCE.md) - 故障排查

### 👨‍🔬 开发工程师
- **开始**: [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) - 代码详解
- **扩展**: [INTEGRATION_GUIDE.md#可选增强功能](./INTEGRATION_GUIDE.md) - 可选功能

---

## 核心概念（30秒）

```
FastPlanner（规划器）
  ↓ 生成轨迹
  /planning/bspline
  ↓
MPC 控制器（跟踪）
  ↓ 计算命令
  /hf_platform/joy_vel
  ↓
hf_platform（执行）
  ↓ 返回位置
  /hf_platform/odom
```

**修改**：将话题从仿真版本 (`/odom`, `/cmd_vel`) 改为实物版本 (`/hf_platform/odom`, `/hf_platform/joy_vel`)

---

## 验证清单（5分钟）

修改后立即验证：

- [ ] 编译成功（无 ERROR）
- [ ] 节点启动（无崩溃）
- [ ] 话题连接（用 `rqt_graph` 检查）
- [ ] 数据流动（用 `rostopic hz` 检查）
- [ ] 底盘运动（观察 hf_platform）

---

## 常见问题速查

| 问题 | 解决 |
|------|------|
| 编译失败 | `rm -rf build devel && catkin_make --only mpc_tracking` |
| 底盘不动 | `rostopic echo /hf_platform/joy_vel` 查看命令 |
| 话题错误 | `rqt_graph` 可视化检查连接 |
| 轨迹不准 | 编辑 `mpc_params.yaml` 调整权重 |

---

## 文档完整列表

| 文档 | 用途 | 重要度 |
|------|------|--------|
| [README_INTEGRATION.md](./README_INTEGRATION.md) | **你在这里** - 快速入门 | ⭐⭐⭐ |
| [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) | 快速参考卡 | ⭐⭐⭐ |
| [INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md) | 详细集成指南 | ⭐⭐ |
| [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) | 代码修改详解 | ⭐⭐ |
| [INTEGRATION_CHECKLIST.md](./INTEGRATION_CHECKLIST.md) | 集成检查清单 | ⭐⭐ |
| [INTEGRATION_SUMMARY.md](./INTEGRATION_SUMMARY.md) | 项目总结 | ⭐ |

---

## 下一步

**立即开始**：
1. 打开 `src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp`
2. 修改 207 和 209 两行
3. 运行 `catkin_make --only mpc_tracking`
4. 启动三个终端
5. 在 RViz 设置目标

**需要帮助**：查看 [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)

---

**预计时间**：5分钟集成 + 10分钟测试 = **15分钟投入使用**

**你已准备好！** ✅ 现在就开始吧！

