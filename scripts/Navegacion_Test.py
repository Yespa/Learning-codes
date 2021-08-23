#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy


#Importaciones generales
import time

import math


from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from std_msgs.msg import String


class Node_navegation_drone:

    def __init__(self):
        
        #PUBLICADORES

        #Publicador de la velocidad calculado para el dron
        self.pub_vel_nav = rospy.Publisher('Nav/vel_lin',TwistStamped,queue_size=10)

    #Metodo para publicar la velocidad del dron
    def publish_velocity(self):

        #Se solicita la información a la pixhawk
        # --> Lleno los vectores de velocidad lineales y angulares
        
        vel_drone = TwistStamped()

        vel_drone.header.stamp = rospy.Time.now()
        vel_drone.header.frame_id = "Nav/Velocidades"
        vel_drone.twist.linear.x = 1
        vel_drone.twist.linear.y = 1
        vel_drone.twist.linear.z = 0
        vel_drone.twist.angular.x = 0
        vel_drone.twist.angular.y = 0
        vel_drone.twist.angular.z = 0

        self.pub_vel_nav.publish(vel_drone)

def main():

    print("Nodo inicializado........")
    rospy.init_node('Navegacion')
    rospy.Rate(25)

    Navegacion = Node_navegation_drone()

    while not rospy.is_shutdown():
        
        Navegacion.publish_velocity()




if __name__ == "__main__":
    main()

print("Script dead")