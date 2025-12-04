# 控制器架构对比：iLQR / MPPI / MPC

## 1. 架构总览

```
                    ┌──────────────────┐
                    │   目标点 (Goal)   │
                    └────────┬─────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│          Layer 1: 全局路径规划 (A*)                              │
│          global_planner.py                                      │
│          输出: 几何路径 [(x,y), ...]                            │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
        ┌────────────────────┴────────────────────┐
        │                                          │
        ▼                                          ▼
┌───────────────────┐                    ┌───────────────────┐
│   二层架构        │                    │   三层架构        │
│   (iLQR/MPPI)    │                    │   (MPC)           │
└───────────────────┘                    └───────────────────┘
```

---

## 2. 二层架构 (iLQR / MPPI)

### 数据流

```
A* 路径 → TrajectoryGenerator (B-Spline平滑) → PathHandler (剪裁) → 控制器
```

### 文件结构

| 文件 | 功能 |
|------|------|
| `ilqr_two_layer.py` | iLQR 二层控制器主程序 |
| `mppi_two_layer.py` | MPPI 二层控制器主程序 |
| `path_handler.py` | 路径处理 (B-Spline平滑 + 剪裁) |
| `local_planner.py` | TrajectoryGenerator (B-Spline + 速度规划) |
| `ilqr_tracker.py` | iLQR 轨迹跟踪核心算法 |
| `mppi_node.py` | MPPI 轨迹跟踪核心算法 |

---

## 3. 三层架构 (MPC)

### 数据流

```
A* 路径 → TrajectoryGenerator (B-Spline平滑) → MPC 跟踪
```

### 文件结构

| 文件 | 功能 |
|------|------|
| `mpc_three_layer.py` | MPC 三层控制器主程序 |
| `local_planner.py` | TrajectoryGenerator (B-Spline + 速度规划) |
| `mpc_tracker.py` | MPC 轨迹跟踪核心算法 |

---

## 4. 算法对比

### 4.1 iLQR (迭代线性二次调节器)

**核心思想**: 迭代求解最优控制问题，通过线性化+二次近似

**算法流程**:
```
1. 初始化控制序列 U
2. 前向传播: 计算轨迹 X = rollout(x0, U)
3. 后向传播: 计算反馈增益 K 和前馈 k
4. Line Search: 找最优步长 α
5. 更新: U_new = U + α*k + K*(X_new - X)
6. 重复 2-5 直到收敛
```

**代价函数**:
```
J = Σ [ (x-x_ref)ᵀ Q (x-x_ref) + (u-u_ref)ᵀ R (u-u_ref) ] + 终端代价
```

**特点**:
- ✅ 计算效率高 (解析解)
- ✅ 收敛快 (二次收敛)
- ❌ 对初值敏感
- ❌ 需要可微动力学

**文件**: `ilqr_tracker.py`

---

### 4.2 MPPI (模型预测路径积分)

**核心思想**: 采样+加权平均，信息论最优控制

**算法流程**:
```
1. 采样 K 条控制序列: U_k = U_mean + noise
2. 前向仿真: 计算每条轨迹 X_k
3. 计算代价: cost_k = Σ running_cost(x, u)
4. 加权平均: U_new = Σ exp(-λ*cost_k) * U_k / Z
5. 输出第一个控制量
```

**模块化 Critics (Nav2 风格)**:
```python
cost = w1 * PathFollowCritic(state)    # 横向偏差
     + w2 * PathAlignCritic(state)     # 朝向对齐
     + w3 * GoalDistCritic(state)      # 目标距离
     + w4 * VelocityCritic(action)     # 速度跟踪
     + w5 * AngularCritic(action)      # 角速度惩罚
     + w6 * ObstacleCritic(state)      # 障碍物
     + w7 * ConstraintCritic(action)   # 软约束
```

**特点**:
- ✅ 无需求导 (采样方法)
- ✅ 天然并行 (GPU加速)
- ✅ 处理非凸代价
- ❌ 采样数量影响质量
- ❌ 计算量大

**文件**: `mppi_node.py`

---

### 4.3 MPC (模型预测控制)

**核心思想**: 滚动时域优化，求解约束非线性规划

**算法流程**:
```
1. 构建优化问题 (CasADi)
2. 设置约束: 动力学、速度、角速度
3. 调用求解器 (IPOPT)
4. 输出第一个控制量
5. 滚动执行
```

**代价函数**:
```
min  Σ [ Q_pos*(x-x_ref)² + Q_theta*(θ-θ_ref)² + R*u² + R_d*(u-u_prev)² ]
s.t. x_{k+1} = f(x_k, u_k)    # 动力学约束
     v_min ≤ v ≤ v_max        # 速度约束
     |w| ≤ w_max              # 角速度约束
```

**特点**:
- ✅ 系统性处理约束
- ✅ 理论保证 (最优性)
- ❌ 计算慢 (非线性求解)
- ❌ 需要凸化或非线性求解器

**文件**: `mpc_tracker.py`

---

## 5. 关键参数对比

| 参数 | iLQR | MPPI | MPC |
|------|------|------|-----|
| 预测步数 N | 30 | 30 | 20 |
| 控制周期 dt | 0.1s | 0.1s | 0.1s |
| 位置权重 Q_pos | 120 | 50 | 15 |
| 角度权重 Q_theta | 1 | 30 | 5 |
| 控制权重 R | 0.1, 0.5 | 1, 5 | 0.05 |
| 采样数 K | - | 1000 | - |
| 最大迭代 | 10 | 1 | IPOPT |

---

## 6. 动力学模型

三种控制器使用相同的差速驱动模型:

```
x_{k+1} = x_k + v * cos(θ) * dt
y_{k+1} = y_k + v * sin(θ) * dt
θ_{k+1} = θ_k + ω * dt
```

状态: `x = [x, y, θ]`
控制: `u = [v, ω]`

---

## 7. 启动命令

```bash
# iLQR 二层
roslaunch simple_mppi run_ilqr_two_layer.launch

# MPPI 二层
roslaunch simple_mppi run_mppi_two_layer.launch

# MPC 三层
roslaunch simple_mppi turtlebot3_mpc_three_layer.launch
```

---

## 8. ROS 话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/global_path` | Path | A* 全局路径 (绿色) |
| `/smooth_path` / `/active_path` | Path | 平滑后轨迹 (蓝色) |
| `/ilqr_predict_path` / `/mppi_predict_path` / `/mpc_predict_path` | Path | 预测轨迹 (红色) |
| `/cmd_vel` | Twist | 控制指令 |
| `/custom_goal` | PoseStamped | 目标点输入 |

---

## 9. 选择建议

| 场景 | 推荐 | 原因 |
|------|------|------|
| 实时性要求高 | iLQR | 计算效率高 |
| GPU 可用 | MPPI | 天然并行 |
| 约束复杂 | MPC | 系统性处理约束 |
| 代价函数非凸 | MPPI | 采样方法不需凸性 |
| 嵌入式部署 | iLQR | 计算量小 |
