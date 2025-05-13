#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def square_test():
    rospy.init_node('square_test')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    side_length = 1.0  # mét
    linear_speed = 0.2  # m/s
    angular_speed = 0.5  # rad/s
    turn_time = math.pi / (2 * angular_speed)  # Thời gian quay 90 độ
    
    for i in range(4):
        # Tiến thẳng
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        pub.publish(move_cmd)
        rospy.loginfo(f"Moving forward side {i+1}")
        time.sleep(side_length / linear_speed)
        
        # Dừng
        move_cmd.linear.x = 0
        pub.publish(move_cmd)
        time.sleep(0.5)
        
        # Quay 90 độ
        move_cmd.angular.z = angular_speed
        pub.publish(move_cmd)
        rospy.loginfo("Turning 90 degrees")
        time.sleep(turn_time)
        
        # Dừng
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        square_test()
    except rospy.ROSInterruptException:
        pass