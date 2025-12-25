#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

HELP = r"""
Ridgeback Omni Teleop (hold publishing)

Movement keys (like teleop_twist_keyboard + strafe):
  i    : forward
  ,    : back
  j    : rotate left
  l    : rotate right
  u/o  : forward + rotate left/right
  m/.  : back    + rotate left/right

Strafe (omni):
  h    : left  (linear.y +)
  n    : right (linear.y -)

Stop / exit:
  k or SPACE : stop immediately (publish zero)
  CTRL-C or x: exit

Speed scaling:
  q / z : increase / decrease linear speed
  w / x : increase / decrease angular speed

Help:
  ? or H : print this help again
"""

MOVE_BINDINGS = {
    'i': ( 1,  0,  0),
    ',': (-1,  0,  0),
    'j': ( 0,  0,  1),
    'l': ( 0,  0, -1),
    'u': ( 1,  0,  1),
    'o': ( 1,  0, -1),
    'm': (-1,  0,  1),
    '.': (-1,  0, -1),
    'h': ( 0,  1,  0),   # strafe left
    'n': ( 0, -1,  0),   # strafe right
}

def get_key(timeout):
    """Non-blocking key read with timeout (seconds)."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def publish_zero(pub):
    pub.publish(Twist())

if __name__ == "__main__":
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("ridgeback_teleop_hold")

    out_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

    rate_hz   = float(rospy.get_param("~rate", 50.0))       # 50Hz 更跟手
    timeout_s = float(rospy.get_param("~timeout", 0.3))     # 超时自动停

    lin_speed = float(rospy.get_param("~lin_speed", 0.3))   # 初始线速度 (m/s)
    ang_speed = float(rospy.get_param("~ang_speed", 0.8))   # 初始角速度 (rad/s)

    lin_step  = float(rospy.get_param("~lin_step", 0.05))   # 每次调速步长
    ang_step  = float(rospy.get_param("~ang_step", 0.1))

    pub = rospy.Publisher(out_topic, Twist, queue_size=1)

    x = y = th = 0
    last_input_time = rospy.Time.now()

    rospy.loginfo("Teleop publishing to %s", out_topic)
    rospy.loginfo("rate=%.1fHz timeout=%.2fs lin_speed=%.2f ang_speed=%.2f",
                  rate_hz, timeout_s, lin_speed, ang_speed)
    print(HELP)

    try:
        tty.setraw(sys.stdin.fileno())
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            # 用 1/rate 的超时时间去等按键：更低CPU、更稳定的响应
            key = get_key(timeout=1.0 / rate_hz)
            if key == '\x03':  # Ctrl+C
                break
            now = rospy.Time.now()

            if key:
                last_input_time = now

                if key in MOVE_BINDINGS:
                    x, y, th = MOVE_BINDINGS[key]

                elif key in ['k', ' ']:
                    x = y = th = 0
                    publish_zero(pub)

                elif key == 'q':
                    lin_speed += lin_step
                    rospy.loginfo("lin_speed=%.2f", lin_speed)
                elif key == 'z':
                    lin_speed = max(0.0, lin_speed - lin_step)
                    rospy.loginfo("lin_speed=%.2f", lin_speed)

                elif key == 'w':
                    ang_speed += ang_step
                    rospy.loginfo("ang_speed=%.2f", ang_speed)
                elif key == 'x':
                    # x 既是退出键也是减角速度键会冲突，所以这里用大写 X 来减速
                    # 小写 x 退出（下面处理）
                    pass
                elif key == 'X':
                    ang_speed = max(0.0, ang_speed - ang_step)
                    rospy.loginfo("ang_speed=%.2f", ang_speed)

                elif key in ['?', 'H']:
                    print(HELP)

                elif key == 'x':
                    break
                else:
                    # 未知键：不改变方向，但会刷新 last_input_time
                    pass

            # 超时自动停（安全）
            if (now - last_input_time).to_sec() > timeout_s:
                x = y = th = 0

            msg = Twist()
            msg.linear.x  = x  * lin_speed
            msg.linear.y  = y  * lin_speed
            msg.angular.z = th * ang_speed
            pub.publish(msg)

            rate.sleep()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        publish_zero(pub)
        rospy.loginfo("Teleop exited, published zero cmd_vel.")
