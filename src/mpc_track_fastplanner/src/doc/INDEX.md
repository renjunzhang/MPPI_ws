# 📚 FastPlanner + MPC 集成文档索引

## 快速导航

### 🟢 如果你只有 5 分钟
👉 **[00_START_HERE.md](./00_START_HERE.md)** - 快速入门指南

### 🟡 如果你有 15 分钟
👉 **[QUICK_REFERENCE.md](./QUICK_REFERENCE.md)** - 快速参考卡

### 🔴 如果你有 1 小时
👉 **[INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md)** - 完整集成指南

---

## 文档详细说明

### 📖 核心文档

#### 1. **00_START_HERE.md** ⭐⭐⭐ 必读
- **时间**: 5 分钟
- **内容**: 快速入门流程、文档导航、常见问题
- **适合**: 所有人（第一个看这个）
- **核心**: 3 步完成集成

#### 2. **QUICK_REFERENCE.md** ⭐⭐⭐ 常用
- **时间**: 15 分钟
- **内容**: 快速参考卡、一键集成脚本、诊断决策树
- **适合**: 运维人员、测试人员
- **核心**: 快速查询和排查

#### 3. **INTEGRATION_GUIDE.md** ⭐⭐ 详细
- **时间**: 30 分钟
- **内容**: 完整集成方案、多个集成方案、优化建议
- **适合**: 系统集成工程师
- **核心**: 深度理解和扩展

#### 4. **CODE_MODIFICATIONS.md** ⭐⭐ 技术
- **时间**: 30 分钟
- **内容**: 代码逐行修改、文件位置、编译配置
- **适合**: 开发工程师
- **核心**: 理解每一行修改

#### 5. **INTEGRATION_CHECKLIST.md** ⭐⭐ 流程
- **时间**: 分阶段
- **内容**: 7 阶段集成检查清单、验证步骤、问题诊断
- **适合**: 项目经理、质量管理
- **核心**: 全流程管理和验收

#### 6. **INTEGRATION_SUMMARY.md** ⭐ 概览
- **时间**: 10 分钟
- **内容**: 项目概览、系统架构、部署清单
- **适合**: 项目经理、技术负责人
- **核心**: 项目全景视图

---

## 按角色推荐阅读顺序

### 👨‍💼 项目经理

1. **必读**: [00_START_HERE.md](./00_START_HERE.md) - 了解全貌
2. **重点**: [INTEGRATION_CHECKLIST.md](./INTEGRATION_CHECKLIST.md) - 管理进度
3. **参考**: [INTEGRATION_SUMMARY.md](./INTEGRATION_SUMMARY.md) - 掌握细节
4. **查询**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 遇到问题时

### 👨‍💻 系统集成工程师

1. **必读**: [00_START_HERE.md](./00_START_HERE.md) - 快速入门
2. **深入**: [INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md) - 完整方案
3. **技术**: [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) - 代码细节
4. **排查**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 问题诊断

### 🔧 运维/测试人员

1. **必读**: [00_START_HERE.md](./00_START_HERE.md) - 快速入门
2. **常用**: [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 日常操作
3. **排查**: [QUICK_REFERENCE.md#快速诊断](./QUICK_REFERENCE.md) - 故障排查
4. **脚本**: [QUICK_REFERENCE.md#一键集成脚本](./QUICK_REFERENCE.md) - 自动化

### 👨‍🔬 开发工程师

1. **必读**: [00_START_HERE.md](./00_START_HERE.md) - 快速入门
2. **技术**: [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) - 代码修改
3. **扩展**: [INTEGRATION_GUIDE.md#可选增强功能](./INTEGRATION_GUIDE.md) - 功能扩展
4. **架构**: [INTEGRATION_GUIDE.md#系统集成清单](./INTEGRATION_GUIDE.md) - 理解架构

---

## 按问题类型查询

### 🤔 我想快速了解项目

**推荐路径**: 
- [00_START_HERE.md](./00_START_HERE.md) (5 分钟)
- [INTEGRATION_SUMMARY.md](./INTEGRATION_SUMMARY.md) (10 分钟)

### 🔨 我要立即开始集成

**推荐路径**:
- [00_START_HERE.md](./00_START_HERE.md) (5 分钟)
- [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - 集成步骤 (5 分钟)
- 开始修改代码 (2 分钟)

### 📋 我需要详细的集成步骤

**推荐路径**:
- [INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md) - 第一步到第五步
- [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) - 每个修改的详解

### ⚠️ 集成出现问题

**推荐路径**:
- [QUICK_REFERENCE.md#快速排查](./QUICK_REFERENCE.md) (5 分钟快速诊断)
- [INTEGRATION_GUIDE.md#第五步](./INTEGRATION_GUIDE.md) (详细排查)
- [INTEGRATION_CHECKLIST.md#阶段-5](./INTEGRATION_CHECKLIST.md) (问题诊断决策树)

### 📊 我要管理项目进度

**推荐路径**:
- [INTEGRATION_CHECKLIST.md](./INTEGRATION_CHECKLIST.md) (完整的检查清单)
- 按照 7 个阶段逐一推进

### 🚀 我想深入理解系统

**推荐路径**:
- [INTEGRATION_SUMMARY.md](./INTEGRATION_SUMMARY.md) (系统概览)
- [INTEGRATION_GUIDE.md](./INTEGRATION_GUIDE.md) (完整指南)
- [CODE_MODIFICATIONS.md](./CODE_MODIFICATIONS.md) (代码细节)

---

## 核心修改速查表

```
要修改的文件: src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp

修改 1 (行 207):
  /cmd_vel          →  /hf_platform/joy_vel

修改 2 (行 209):
  /odom             →  /hf_platform/odom

编译:
  catkin_make --only mpc_tracking

运行:
  rosrun mpc_tracking mpc_tracking_node
```

---

## 文档统计

| 文档 | 大小 | 阅读时间 | 更新日期 |
|------|------|--------|--------|
| 00_START_HERE.md | ~2KB | 5 分钟 | 2025-11-14 |
| QUICK_REFERENCE.md | ~8KB | 15 分钟 | 2025-11-14 |
| INTEGRATION_GUIDE.md | ~12KB | 30 分钟 | 2025-11-14 |
| CODE_MODIFICATIONS.md | ~10KB | 30 分钟 | 2025-11-14 |
| INTEGRATION_CHECKLIST.md | ~15KB | 分阶段 | 2025-11-14 |
| INTEGRATION_SUMMARY.md | ~12KB | 15 分钟 | 2025-11-14 |
| **总计** | **~60KB** | **~2小时** | - |

---

## 快速命令参考

```bash
# 修改代码
vim src/mpc_track_fastplanner/src/mpc_tracking/src/mpc_node.cpp

# 编译
cd /home/a/hf_move && catkin_make --only mpc_tracking

# 验证编译
ls -la devel/lib/mpc_tracking/mpc_tracking_node

# 启动系统
roslaunch hf_bringup bringup.launch &
roslaunch plan_manage kino.launch &
rosrun mpc_tracking mpc_tracking_node &

# 监控状态
rqt_graph                             # 可视化节点
rostopic hz /planning/bspline         # 规划频率
rostopic echo /hf_platform/joy_vel    # 控制命令
```

---

## 常见问题快速查询

| 问题 | 查看文档 | 位置 |
|------|--------|------|
| 5分钟快速入门 | QUICK_REFERENCE.md | [集成流程](#) |
| 详细步骤 | INTEGRATION_GUIDE.md | [第四步](#) |
| 代码怎么改 | CODE_MODIFICATIONS.md | [修改1-7](#) |
| 编译失败 | QUICK_REFERENCE.md | [问题1](#) |
| 底盘不动 | QUICK_REFERENCE.md | [问题2](#) |
| 轨迹不准 | QUICK_REFERENCE.md | [问题3](#) |
| 完整流程 | INTEGRATION_CHECKLIST.md | [阶段1-7](#) |

---

## 最后一步

你已经看完文档索引。现在：

1. **选择你的角色** - 按照上面的"按角色推荐"选择
2. **阅读首个文档** - 通常是 `00_START_HERE.md`
3. **开始集成** - 按照文档步骤一步步进行
4. **遇到问题** - 查询"按问题类型查询"部分

---

**提示**: 如果你不确定从哪里开始，就读 **[00_START_HERE.md](./00_START_HERE.md)** 👈

---

**版本**: v1.0  
**最后更新**: 2025年11月14日  
**维护**: 项目技术团队

