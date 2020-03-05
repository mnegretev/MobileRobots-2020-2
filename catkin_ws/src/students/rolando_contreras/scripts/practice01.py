#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-2
# PRACTICE 1 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot along a path describing a 2mx2m square.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
import tf
from std_msgs.msg import Float32MultiArray

NAME = "Rolando_Contreras"

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return None

        #
        # TODO:
        # Declare a Float32MultiArray message and assign the appropiate speeds:
        # [sl, sr] where sl is the left tire speed and sr, the right tire speed, both in m/s
        # Calculate the speeds to move the robot describing a 2mx2m square.
        # You can do it in open or closed loop. For the latter case, you can use the
        # get_robot_pose function to get the current robot configuration.
        # Publish the message.
        # You can declare as many variables as you need.
        #

def main():
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    pub_speeds = rospy.Publisher("/rotombot/hardware/motor_speeds", Float32MultiArray, queue_size=10)
    loop = rospy.Rate(20)
    listener = tf.TransformListener()
    msgM = Float32MultiArray()
    rd = 0.5
    ri = 0.5
    a = 150
    b = 30
    time_move = a
    time_turn = b
    
    while not rospy.is_shutdown() and time_move >=0:

   	msgM.data=[rd,ri]
    	pub_speeds.publish(msgM)
   	time_move = time_move - 1
        loop.sleep()
        time_move = time_move - 1
	msgM.data=[0,0]
	pub_speeds.publish(msgM)

    while not rospy.is_shutdown() and time_turn >=0:
	msgM.data=[-rd,ri]
    	pub_speeds.publish(msgM)
   	time_turn = time_turn - 1
	loop.sleep()
	time_turn = time_turn - 1
        msgM.data=[0,0]
	pub_speeds.publish(msgM)
	
    time_move = a
    while not rospy.is_shutdown() and time_move >=0:

   	msgM.data=[rd,ri]
    	pub_speeds.publish(msgM)
   	time_move = time_move - 1
        loop.sleep()
        time_move = time_move - 1
	msgM.data=[0,0]
	pub_speeds.publish(msgM)

    time_turn = b
    while not rospy.is_shutdown() and time_turn >=0:
	msgM.data=[-rd,ri]
    	pub_speeds.publish(msgM)
   	time_turn = time_turn - 1
	loop.sleep()
	time_turn = time_turn - 1
        msgM.data=[0,0]
	pub_speeds.publish(msgM)

    time_move = a
    while not rospy.is_shutdown() and time_move >=0:

   	msgM.data=[rd,ri]
    	pub_speeds.publish(msgM)
   	time_move = time_move - 1
        loop.sleep()
        time_move = time_move - 1
	msgM.data=[0,0]
	pub_speeds.publish(msgM)
	
    time_turn = b
    while not rospy.is_shutdown() and time_turn >=0:
	msgM.data=[-rd,ri]
    	pub_speeds.publish(msgM)
   	time_turn = time_turn - 1
	loop.sleep()
	time_turn = time_turn - 1
        msgM.data=[0,0]
	pub_speeds.publish(msgM)

    time_move = a
    while not rospy.is_shutdown() and time_move >=0:

   	msgM.data=[rd,ri]
    	pub_speeds.publish(msgM)
   	time_move = time_move - 1
        loop.sleep()
        time_move = time_move - 1
	msgM.data=[0,0]
	pub_speeds.publish(msgM)

    time_turn = b
    while not rospy.is_shutdown() and time_turn >=0:
	msgM.data=[-rd,ri]
    	pub_speeds.publish(msgM)
   	time_turn = time_turn - 1
	loop.sleep()
	time_turn = time_turn - 1
        msgM.data=[0,0]
	pub_speeds.publish(msgM)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

