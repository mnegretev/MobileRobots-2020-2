#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float32MultiArray

def callback_speeds(msg):
    global control_v, control_w
    D = 0.5
    control_v = (msg.data[0] + msg.data[1])/2
    control_w = (msg.data[1] - msg.data[0])/D

def main():
    global control_v, control_w
    fs = 20
    
    print "INITIALIZING BASE SIMULATOR ..."
    rospy.init_node("base_simul")
    rospy.Subscriber("/rotombot/hardware/motor_speeds", Float32MultiArray, callback_speeds)
    loop = rospy.Rate(fs)
    br = tf.TransformBroadcaster()

    control_v = 0
    control_w = 0
    robot_x = 0
    robot_y = 0
    robot_a = 0
    while not rospy.is_shutdown():
        robot_x += 1.0/fs * control_v * math.cos(robot_a)
        robot_y += 1.0/fs * control_v * math.sin(robot_a)
        robot_a += 1.0/fs * control_w
        br.sendTransform((robot_x, robot_y, 0), tf.transformations.quaternion_from_euler(0,0,robot_a),
                         rospy.Time.now(), "base_link", "odom")
        loop.sleep()

if __name__ == '__main__':
    main()
