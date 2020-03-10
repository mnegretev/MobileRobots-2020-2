#!/usr/bin/env python
# -*- coding: utf-8 -*-
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-2
# PRACTICE 1 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot along a path describing a 2mx2m square.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
import tf
import time
from std_msgs.msg import Float32MultiArray

NAME = "MIRAMONTES_SARABIA_LUIS_ENRIQUE"

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
    msgToSend = Float32MultiArray()
    time.sleep(2) #Espero dos segundos, sólo para estar listo
    while not rospy.is_shutdown():
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
        msgToSend.data = [0.5, 0.5]
        pub_speeds.publish(msgToSend)
        time.sleep(4) #Se avanza dos metros en cuatro segundos 
        msgToSend.data = [-0.5, 0.5]
        pub_speeds.publish(msgToSend)
        '''
        calculando la velocidad angular a partir de la velocidad tangencial y usando 0.5m
        como diámetro (medido dentro del simulador, a no tener medida entre llantas se
        uso el diámetro completo del robot) se obtiene una velocidad tangencial de 2rad/s
        para girar 90° se requiere un giro de 1.57 rad, lo que nos da como resultado
        un tiempo de giro de 0.785 s, redondeo a 0.79 para mantener congruente el redondeo del diámetro..
        La medida no es perfecta, después de varias vueltas el robot se comienza a desviar un poco,
        pero es lo más que me puedo aproximar sin cucharear tiempos y usando matemáticas.
        '''
        time.sleep(0.79)
        loop.sleep()
        #El loop seguirá.


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

