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

NAME = "HERNANDEZ_PENAFORT"

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

def main():
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    pub_speeds = rospy.Publisher("/rotombot/hardware/motor_speeds", Float32MultiArray, queue_size=10)
    loop = rospy.Rate(20)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
       
        msg = Float32MultiArray()
        msg.data = [0,0] 
        x, y, angInicial = get_robot_pose(listener)

        if angInicial > math.pi * 2:
            angInicial -= math.pi * 2
        if angInicial < 0:
            angInicial += 2 * math.pi
        linear = True
        startTime = time.time()
        while True == True:
            timeActual = time.time()
            if timeActual > startTime + 4:
                linear = False
                starTime = timeActual
            if linear == True:
                msg.data = [0.5,0.5]
            else:
                print(angActual, angInicial)
                x, y, angActual = get_robot_pose(listener)
                if angInicial > math.pi * 2:
                    angInicial -= math.pi * 2
                if angActual > math.pi * 2:
                    angActual -= 2 * math.pi
                if angActual < 0:
                    angActual += 2 * math.pi
                if angActual > angInicial + math.pi/2:
                    print("Entro")
                    angInicial = angActual - 0.05
                    linear = True
                    msg.data = [0, 0]
                    startTime = timeActual
                else:
                    msg.data = [0, 0.4]

            pub_speeds.publish(msg)
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
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

