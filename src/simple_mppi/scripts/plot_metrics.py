#!/usr/bin/env python3
"""
性能指标对比工具
================

功能:
    1. 订阅 /nav_metrics 和 /nav_algo_name
    2. 收集三种算法 (MPC, MPPI, iLQR) 的结果
    3. 收集完毕后自动生成对比报告
    4. 支持保存到 CSV

使用方式:
    1. 在终端1运行: rosrun simple_mppi plot_metrics.py
    2. 在终端2分别运行三个 launch 文件
    3. 每次发布目标点 (0.67, 0.65) 后等待导航完成
    4. 收集到三个结果后自动输出对比

指标格式 (Float32MultiArray, 16 字段):
    [path_length, avg_speed, avg_cte, max_cte,
     jerk_v, jerk_w, v_variance, w_variance, control_effort,
     total_time, efficiency, avg_solve_time,
     heading_error, oscillation_count, stuck_time, success]
"""
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String
from datetime import datetime


class MetricsComparator:
    """三种架构性能对比器"""

    # 指标名称映射
    METRIC_NAMES = [
        'path_length', 'avg_speed', 'avg_cte', 'max_cte',
        'jerk_v', 'jerk_w', 'v_variance', 'w_variance', 'control_effort',
        'total_time', 'efficiency', 'avg_solve_time',
        'heading_error', 'oscillation_count', 'stuck_time', 'success'
    ]

    # 期望收集的算法
    EXPECTED_ALGOS = {'MPC', 'MPPI', 'iLQR'}

    def __init__(self):
        rospy.init_node('metrics_comparator')
        
        self.records = {}  # {algo_name: metrics_dict}
        self.current_algo = None
        
        # 订阅
        rospy.Subscriber('/nav_algo_name', String, self._algo_cb)
        rospy.Subscriber('/nav_metrics', Float32MultiArray, self._metrics_cb)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("  性能指标对比工具")
        rospy.loginfo("=" * 50)
        rospy.loginfo("  等待收集三种算法的结果...")
        rospy.loginfo("  已收集: 0/3")
        rospy.loginfo("=" * 50)

    def _algo_cb(self, msg):
        """接收算法名称"""
        self.current_algo = msg.data
        rospy.loginfo(f"[Comparator] 接收到算法: {self.current_algo}")

    def _metrics_cb(self, msg):
        """接收指标数据"""
        if self.current_algo is None:
            rospy.logwarn("[Comparator] 未收到算法名称，忽略此次数据")
            return
        
        if len(msg.data) < 16:
            rospy.logwarn(f"[Comparator] 数据格式错误，期望 16 字段，收到 {len(msg.data)}")
            return
        
        # 解析指标
        metrics = {}
        for i, name in enumerate(self.METRIC_NAMES):
            metrics[name] = msg.data[i]
        
        # 存储
        self.records[self.current_algo] = metrics
        
        rospy.loginfo(f"[Comparator] 已记录 {self.current_algo} 的结果")
        rospy.loginfo(f"[Comparator] 已收集: {len(self.records)}/3 ({', '.join(self.records.keys())})")
        
        # 检查是否收集完毕
        if self._check_complete():
            self._print_comparison()
            self._save_csv()

    def _check_complete(self):
        """检查是否收集完三种算法"""
        return self.EXPECTED_ALGOS.issubset(set(self.records.keys()))

    def _print_comparison(self):
        """打印对比报告"""
        print("\n")
        print("=" * 80)
        print("                    三种架构性能对比报告")
        print("=" * 80)
        
        # 基本信息
        print(f"\n📅 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"🎯 目标: (0.67, 0.65)")
        
        # === 成功率 ===
        print("\n" + "─" * 80)
        print("  ✓ 成功状态")
        print("─" * 80)
        for algo in ['MPC', 'MPPI', 'iLQR']:
            success = self.records[algo]['success']
            status = "✅ 成功" if success > 0.5 else "❌ 失败"
            print(f"  {algo:8} {status}")
        
        # === 效率指标 ===
        print("\n" + "─" * 80)
        print("  📊 效率指标")
        print("─" * 80)
        self._print_metric_row('路径长度 (m)', 'path_length', '{:.3f}', lower_better=True)
        self._print_metric_row('总时间 (s)', 'total_time', '{:.2f}', lower_better=True)
        self._print_metric_row('平均速度 (m/s)', 'avg_speed', '{:.3f}', lower_better=False)
        self._print_metric_row('路径效率 (%)', 'efficiency', '{:.1f}', lower_better=False, scale=100)
        
        # === 跟踪质量 ===
        print("\n" + "─" * 80)
        print("  🎯 跟踪质量")
        print("─" * 80)
        self._print_metric_row('平均 CTE (cm)', 'avg_cte', '{:.2f}', lower_better=True, scale=100)
        self._print_metric_row('最大 CTE (cm)', 'max_cte', '{:.2f}', lower_better=True, scale=100)
        self._print_metric_row('朝向误差 (°)', 'heading_error', '{:.1f}', lower_better=True, scale=57.3)
        
        # === 控制平滑性 ===
        print("\n" + "─" * 80)
        print("  🎛️  控制平滑性")
        print("─" * 80)
        self._print_metric_row('速度方差', 'v_variance', '{:.4f}', lower_better=True)
        self._print_metric_row('角速度方差', 'w_variance', '{:.4f}', lower_better=True)
        self._print_metric_row('Jerk(v)', 'jerk_v', '{:.4f}', lower_better=True)
        self._print_metric_row('Jerk(w)', 'jerk_w', '{:.4f}', lower_better=True)
        self._print_metric_row('震荡次数', 'oscillation_count', '{:.0f}', lower_better=True)
        
        # === 计算效率 ===
        print("\n" + "─" * 80)
        print("  ⚡ 计算效率")
        print("─" * 80)
        self._print_metric_row('平均求解 (ms)', 'avg_solve_time', '{:.2f}', lower_better=True)
        self._print_metric_row('卡住时间 (s)', 'stuck_time', '{:.2f}', lower_better=True)
        
        # === 综合评分 ===
        print("\n" + "─" * 80)
        print("  🏆 综合评分 (越高越好)")
        print("─" * 80)
        scores = self._compute_scores()
        max_score = max(scores.values())
        for algo in ['MPC', 'MPPI', 'iLQR']:
            score = scores[algo]
            medal = "🥇" if score == max_score else "  "
            bar = "█" * int(score / 10) + "░" * (10 - int(score / 10))
            print(f"  {medal} {algo:8} {bar} {score:.1f}")
        
        print("\n" + "=" * 80)
        print("                         对比完成！")
        print("=" * 80 + "\n")

    def _print_metric_row(self, label, key, fmt, lower_better=True, scale=1.0):
        """打印一行指标对比"""
        values = {}
        for algo in ['MPC', 'MPPI', 'iLQR']:
            values[algo] = self.records[algo][key] * scale
        
        # 找最优值
        if lower_better:
            best_val = min(values.values())
        else:
            best_val = max(values.values())
        
        # 格式化输出
        line = f"  {label:16}"
        for algo in ['MPC', 'MPPI', 'iLQR']:
            val = values[algo]
            formatted = fmt.format(val)
            if abs(val - best_val) < 1e-6:
                line += f"  {algo}: {formatted:>10} 🏆"
            else:
                line += f"  {algo}: {formatted:>10}   "
        print(line)

    def _compute_scores(self):
        """计算综合评分 (0-100)"""
        scores = {}
        
        for algo in ['MPC', 'MPPI', 'iLQR']:
            m = self.records[algo]
            
            # 各项评分 (归一化到 0-100)
            # 跟踪质量 (40%)
            cte_score = max(0, 100 - m['avg_cte'] * 1000)  # CTE 越小越好
            heading_score = max(0, 100 - np.degrees(m['heading_error']) * 5)
            tracking = (cte_score * 0.6 + heading_score * 0.4) * 0.4
            
            # 效率 (30%)
            efficiency_score = m['efficiency'] * 100
            time_score = max(0, 100 - m['total_time'] * 2)
            efficiency = (efficiency_score * 0.6 + time_score * 0.4) * 0.3
            
            # 平滑性 (20%)
            osc_score = max(0, 100 - m['oscillation_count'] * 2)
            var_score = max(0, 100 - m['w_variance'] * 500)
            smoothness = (osc_score * 0.5 + var_score * 0.5) * 0.2
            
            # 计算效率 (10%)
            solve_score = max(0, 100 - m['avg_solve_time'] * 2)
            compute = solve_score * 0.1
            
            scores[algo] = tracking + efficiency + smoothness + compute
        
        return scores

    def _save_csv(self):
        """保存结果到 CSV"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"/tmp/nav_comparison_{timestamp}.csv"
        
        with open(filename, 'w') as f:
            # 表头
            f.write("Metric," + ",".join(['MPC', 'MPPI', 'iLQR']) + "\n")
            
            # 数据行
            for name in self.METRIC_NAMES:
                values = [str(self.records[algo][name]) for algo in ['MPC', 'MPPI', 'iLQR']]
                f.write(f"{name},{','.join(values)}\n")
        
        rospy.loginfo(f"[Comparator] 结果已保存到: {filename}")

    def run(self):
        """运行"""
        rospy.spin()


def main():
    try:
        comparator = MetricsComparator()
        comparator.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
