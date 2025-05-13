#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import math

x = 0.0
y = 0.0
th = 0.0
last_time = None

left_vel = None
right_vel = None

pub = None
odom_broadcaster = None

def vel_callback_left(msg):
    global left_vel
    left_vel = max(min(msg.data, 200.0), -200.0)  # giới hạn an toàn

def vel_callback_right(msg):
    global right_vel
    right_vel = max(min(msg.data, 200.0), -200.0)

def compute_odometry():
    global x, y, th, last_time, left_vel, right_vel

    if left_vel is None or right_vel is None:
        return

    current_time = rospy.Time.now()
    current_time_sec = current_time.to_sec()

    if last_time is None:
        last_time = current_time_sec
        return

    dt = current_time_sec - last_time
    if dt > 0.2:  # giới hạn dt tối đa
        dt = 0.2
    last_time = current_time_sec

    # Wheel parameters
    wheel_base = 0.32
    radius = 0.05

    # Convert RPM to m/s
    vL = -(left_vel * 2 * math.pi * radius) / 60.0
    vR = -(right_vel * 2 * math.pi * radius) / 60.0

    vx = (vR + vL) / 2.0
    vth = (vR - vL) / wheel_base

    delta_x = vx * math.cos(th) * dt
    delta_y = vx * math.sin(th) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th
    th = math.atan2(math.sin(th), math.cos(th))  # normalize

    # TF transform: odom → base_footprint
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_broadcaster.sendTransform(
        (x, y, 0.0),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # Odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = Quaternion(*odom_quat)

    odom.twist.twist.linear.x = vx
    odom.twist.twist.angular.z = vth

    # Covariance cho SLAM tin tưởng hơn
    odom.pose.covariance = [
        0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 99999, 0, 0, 0,
        0, 0, 0, 99999, 0, 0,
        0, 0, 0, 0, 99999, 0,
        0, 0, 0, 0, 0, 0.05
    ]
    odom.twist.covariance = odom.pose.covariance

    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('odometry_node')
    rospy.Subscriber("wheel_left_vel", Float32, vel_callback_left)
    rospy.Subscriber("wheel_right_vel", Float32, vel_callback_right)

    pub = rospy.Publisher("odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        compute_odometry()
        rate.sleep()