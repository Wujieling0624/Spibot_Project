#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import sys, select, tty, termios

# 获取键盘输入的函数
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def move_mode(mode):
    return Int32(mode)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop_node')
    pub = rospy.Publisher('/spibot_gazebo/move_mode', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    print("keyboard_teleop_node is set, please press 'w' 's' 'a' 'd' 'q' 'e' to sent message or press Ctrl+C to quit.")

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if (key == '\x00') or (key == '\x03'):  # Ctrl+C
                break
            elif key == 'w':
                pub.publish(move_mode(1))  # 假设1代表前进
            elif key == 's':
                pub.publish(move_mode(2))  # 假设2代表后退
            elif key == 'a':
                pub.publish(move_mode(3))  # 假设3代表左转
            elif key == 'd':
                pub.publish(move_mode(4))  # 假设4代表右转
            elif key == 'q':
                pub.publish(move_mode(5))  # 假设5代表上升或其他模式
            elif key == 'e':
                pub.publish(move_mode(6))  # 假设6代表下降或其他模式
            else:
                pub.publish(move_mode(0))  # 0代表停止或无效输入

            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)