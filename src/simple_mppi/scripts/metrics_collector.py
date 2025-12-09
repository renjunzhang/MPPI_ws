#!/usr/bin/env python3
"""
性能指标收集器 - 共享模块
==========================

为三种架构 (MPC/MPPI/iLQR) 提供统一的指标收集和发布接口。

指标包括:
    - 轨迹跟踪质量: path_length, avg_cte, max_cte, avg_speed
    - 控制平滑性: jerk_v, jerk_w, v_variance, w_variance, control_effort
    - 效率指标: total_time, efficiency, avg_solve_time
    - 稳定性指标: heading_error, oscillation_count, stuck_time, success

消息格式 (Float32MultiArray, 16 字段):
    [path_length, avg_speed, avg_cte, max_cte,
     jerk_v, jerk_w, v_variance, w_variance, control_effort,
     total_time, efficiency, avg_solve_time,
     heading_error, oscillation_count, stuck_time, success]
"""

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, String


class MetricsCollector:
    """
    性能指标收集器

    Usage:
        collector = MetricsCollector(algo_name="MPPI")
        collector.set_goal(start_pos, goal_pos)
        
        # 每帧调用
        collector.record(pose, control, ref_point, solve_time)
        
        # 到达目标后
        collector.finish(success=True)
    """

    def __init__(self, algo_name="unknown"):
        """
        Args:
            algo_name: 算法名称 (MPPI, MPC, iLQR)
        """
        self.algo_name = algo_name
        
        # 发布器
        self.pub_metrics = rospy.Publisher('/nav_metrics', Float32MultiArray, queue_size=1, latch=True)
        self.pub_algo_name = rospy.Publisher('/nav_algo_name', String, queue_size=1, latch=True)
        
        self.reset()

    def reset(self):
        """重置所有记录"""
        # 位置记录
        self.poses = []           # [(x, y), ...]
        
        # 控制记录
        self.vs = []              # 线速度
        self.ws = []              # 角速度
        
        # 跟踪误差
        self.ctes = []            # 横向跟踪误差
        self.heading_errors = []  # 朝向误差
        
        # 时间记录
        self.solve_times = []     # 每帧求解时间
        self.start_time = None
        self.end_time = None
        
        # 目标信息
        self.start_pos = None
        self.goal_pos = None

    def set_goal(self, start_pos, goal_pos):
        """
        设置新目标，重置记录
        
        Args:
            start_pos: 起点 (x, y)
            goal_pos: 终点 (x, y)
        """
        self.reset()
        self.start_pos = np.array(start_pos[:2])
        self.goal_pos = np.array(goal_pos[:2])
        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"[Metrics:{self.algo_name}] Start recording")

    def record(self, pose, control, ref_point=None, solve_time=None):
        """
        记录单帧数据
        
        Args:
            pose: 当前位姿 (x, y, theta)
            control: 控制量 (v, w)
            ref_point: 参考点 {'x', 'y', 'theta'} 或 (x, y, theta)，可选
            solve_time: 求解时间 (秒)，可选
        """
        if self.start_time is None:
            return
            
        # 位置
        self.poses.append((pose[0], pose[1]))
        
        # 控制量
        v = float(control[0])
        w = float(control[1])
        self.vs.append(v)
        self.ws.append(w)
        
        # 横向误差 (CTE)
        if ref_point is not None:
            if isinstance(ref_point, dict):
                ref_x, ref_y = ref_point['x'], ref_point['y']
                ref_theta = ref_point.get('theta', pose[2])
            else:
                ref_x, ref_y = ref_point[0], ref_point[1]
                ref_theta = ref_point[2] if len(ref_point) > 2 else pose[2]
            
            # CTE: 到参考点的距离
            cte = np.sqrt((pose[0] - ref_x)**2 + (pose[1] - ref_y)**2)
            self.ctes.append(cte)
            
            # 朝向误差
            heading_err = abs(self._angle_diff(pose[2], ref_theta))
            self.heading_errors.append(heading_err)
        
        # 求解时间
        if solve_time is not None:
            self.solve_times.append(solve_time)

    def finish(self, success=True):
        """
        导航结束，计算并发布指标
        
        Args:
            success: 是否成功到达目标
        """
        self.end_time = rospy.Time.now().to_sec()
        
        if len(self.poses) < 2:
            rospy.logwarn(f"[Metrics:{self.algo_name}] Not enough data")
            return
        
        metrics = self._compute_metrics(success)
        self._publish(metrics)
        self._print_summary(metrics)

    def _compute_metrics(self, success):
        """计算所有指标"""
        metrics = {}
        
        # === 基础指标 ===
        # 路径长度
        path_length = sum(
            np.sqrt((self.poses[i+1][0] - self.poses[i][0])**2 +
                    (self.poses[i+1][1] - self.poses[i][1])**2)
            for i in range(len(self.poses) - 1)
        )
        metrics['path_length'] = path_length
        metrics['avg_speed'] = np.mean(self.vs) if self.vs else 0.0
        metrics['avg_cte'] = np.mean(self.ctes) if self.ctes else 0.0
        metrics['max_cte'] = np.max(self.ctes) if self.ctes else 0.0
        
        # === 控制平滑性 ===
        # Jerk (加加速度) - 使用有限差分
        if len(self.vs) >= 3:
            accels_v = np.diff(self.vs)  # 加速度
            jerks_v = np.diff(accels_v)  # 加加速度
            metrics['jerk_v'] = np.mean(np.abs(jerks_v))
        else:
            metrics['jerk_v'] = 0.0
            
        if len(self.ws) >= 3:
            accels_w = np.diff(self.ws)
            jerks_w = np.diff(accels_w)
            metrics['jerk_w'] = np.mean(np.abs(jerks_w))
        else:
            metrics['jerk_w'] = 0.0
        
        metrics['v_variance'] = np.var(self.vs) if self.vs else 0.0
        metrics['w_variance'] = np.var(self.ws) if self.ws else 0.0
        metrics['control_effort'] = sum(v**2 + w**2 for v, w in zip(self.vs, self.ws))
        
        # === 效率指标 ===
        metrics['total_time'] = self.end_time - self.start_time if self.start_time else 0.0
        
        # 效率 = 直线距离 / 实际路径
        if self.start_pos is not None and self.goal_pos is not None:
            straight_dist = np.linalg.norm(self.goal_pos - self.start_pos)
            metrics['efficiency'] = straight_dist / path_length if path_length > 0 else 0.0
        else:
            metrics['efficiency'] = 0.0
            
        metrics['avg_solve_time'] = np.mean(self.solve_times) * 1000 if self.solve_times else 0.0  # ms
        
        # === 稳定性指标 ===
        metrics['heading_error'] = np.mean(self.heading_errors) if self.heading_errors else 0.0
        
        # 震荡次数 (角速度过零次数)
        oscillations = 0
        for i in range(1, len(self.ws)):
            if self.ws[i-1] * self.ws[i] < 0:  # 符号变化
                oscillations += 1
        metrics['oscillation_count'] = oscillations
        
        # 卡住时间 (v < 0.01 的累计时间)
        dt = 0.1  # 假设 10Hz
        stuck_frames = sum(1 for v in self.vs if abs(v) < 0.01)
        metrics['stuck_time'] = stuck_frames * dt
        
        metrics['success'] = 1.0 if success else 0.0
        
        return metrics

    def _publish(self, metrics):
        """发布指标消息"""
        # 发布算法名称
        self.pub_algo_name.publish(String(data=self.algo_name))
        
        # 发布指标数组
        msg = Float32MultiArray()
        msg.data = [
            metrics['path_length'],
            metrics['avg_speed'],
            metrics['avg_cte'],
            metrics['max_cte'],
            metrics['jerk_v'],
            metrics['jerk_w'],
            metrics['v_variance'],
            metrics['w_variance'],
            metrics['control_effort'],
            metrics['total_time'],
            metrics['efficiency'],
            metrics['avg_solve_time'],
            metrics['heading_error'],
            metrics['oscillation_count'],
            metrics['stuck_time'],
            metrics['success']
        ]
        self.pub_metrics.publish(msg)

    def _print_summary(self, m):
        """打印性能汇总"""
        print("\n" + "="*50)
        print(f"  {self.algo_name} 性能汇总")
        print("="*50)
        print(f"  ✓ 成功: {'是' if m['success'] else '否'}")
        print(f"  📏 路径长度: {m['path_length']:.3f} m")
        print(f"  ⏱️  总时间: {m['total_time']:.2f} s")
        print(f"  🚀 平均速度: {m['avg_speed']:.3f} m/s")
        print(f"  📐 效率: {m['efficiency']*100:.1f}%")
        print("-"*50)
        print(f"  📊 跟踪误差:")
        print(f"     平均 CTE: {m['avg_cte']*100:.2f} cm")
        print(f"     最大 CTE: {m['max_cte']*100:.2f} cm")
        print(f"     平均朝向误差: {np.degrees(m['heading_error']):.1f}°")
        print("-"*50)
        print(f"  🎛️  控制平滑性:")
        print(f"     速度方差: {m['v_variance']:.4f}")
        print(f"     角速度方差: {m['w_variance']:.4f}")
        print(f"     Jerk(v): {m['jerk_v']:.4f}")
        print(f"     Jerk(w): {m['jerk_w']:.4f}")
        print(f"     震荡次数: {int(m['oscillation_count'])}")
        print("-"*50)
        print(f"  ⚡ 计算效率:")
        print(f"     平均求解: {m['avg_solve_time']:.2f} ms")
        print(f"     卡住时间: {m['stuck_time']:.2f} s")
        print("="*50 + "\n")

    @staticmethod
    def _angle_diff(a, b):
        """计算角度差 (归一化到 [-pi, pi])"""
        diff = a - b
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
