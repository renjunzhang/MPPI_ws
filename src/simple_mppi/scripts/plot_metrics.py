#!/usr/bin/env python3
"""
性能指标记录模块
订阅 /nav_metrics，记录并打印导航结果
"""
import rospy
from std_msgs.msg import Float32MultiArray

class MetricsRecorder:
    def __init__(self):
        rospy.init_node('metrics_recorder')
        self.records = []
        self.current_algo = rospy.get_param('~algo', 'unknown')
        rospy.Subscriber('/nav_metrics', Float32MultiArray, self._cb)
        rospy.loginfo(f"[MetricsRecorder] Listening /nav_metrics, algo={self.current_algo}")

    def _cb(self, msg):
        if len(msg.data) >= 4:
            rec = {
                'algo': self.current_algo,
                'path_len': msg.data[0],
                'avg_speed': msg.data[1],
                'avg_cte': msg.data[2],
                'max_cte': msg.data[3]
            }
            self.records.append(rec)
            self._print_record(rec)

    def _print_record(self, r):
        print(f"\n===== {r['algo']} =====")
        print(f"  路径长度: {r['path_len']:.3f} m")
        print(f"  平均速度: {r['avg_speed']:.3f} m/s")
        print(f"  平均横向误差: {r['avg_cte']:.4f} m")
        print(f"  最大横向误差: {r['max_cte']:.4f} m")

    def print_summary(self):
        if not self.records:
            print("No data recorded.")
            return
        print("\n" + "="*50)
        print("汇总")
        print("="*50)
        print(f"{'算法':<10} {'路径(m)':<10} {'速度(m/s)':<12} {'平均CTE(m)':<12} {'最大CTE(m)':<12}")
        print("-"*50)
        for r in self.records:
            print(f"{r['algo']:<10} {r['path_len']:<10.3f} {r['avg_speed']:<12.3f} {r['avg_cte']:<12.4f} {r['max_cte']:<12.4f}")

def main():
    recorder = MetricsRecorder()
    rospy.on_shutdown(recorder.print_summary)
    rospy.spin()

if __name__ == '__main__':
    main()
