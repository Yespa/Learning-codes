#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy


#Importaciones generales
import time

import math


from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from std_msgs.msg import String


class Node_navegation_drone:

    def __init__(self):
        
        self.longitude_now = 0
        self.latitude_now = 0
        self.altitude_now = 0

        #SUSCRIPTORES

        #Suscriptor de la posición actual del drone.
        self.sub_pos_gps = rospy.Subscriber('drone/pos_gps',NavSatFix,self.update_pos_gps)

        #Suscriptor del cabeceo actual del drone
        self.sub_orient_angle_z_now = rospy.Subscriber('drone/orient_angle_z_now', PointStamped, self.update_angle_z)


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
        vel_drone.twist.linear.x = 0
        vel_drone.twist.linear.y = 0
        vel_drone.twist.linear.z = 0
        vel_drone.twist.angular.x = 0
        vel_drone.twist.angular.y = 0
        vel_drone.twist.angular.z = 0

        self.pub_vel_nav.publish(vel_drone)

    #METODO PARA ACTUALIZAR EL VALOR DE LA COORDENADA ENVIADA DESDE EL NODO DEL DRON
    def update_pos_gps(self,msg):
        self.latitude_now = msg.latitude
        self.longitude_now = msg.longitude
        self.altitude_now = msg.altitude

    def update_angle_z(self,point_ang_z):
        self.angle_now = point_ang_z.point.z


    #METODO PARA PARA IR A LA COORDENADA INGRESADA.
    def goto(self):
        
        latitude_destino = -35.3602602 * 10000
        longitude_destino = 149.1652347* 10000

        dist_latitude = (latitude_destino - self.latitude_now*10000)  #Distancia variable a recorrer en latitud
        dist_longitude = (longitude_destino - self.longitude_now*10000) #Distancia variabale a recorrer en longitud
        dist_recorrer = math.sqrt(((dist_latitude)**2)+((dist_longitude)**2)) #Distancia más corta entre la latitud y longitud variable

        ang_rotacion = math.degrees(math.atan2(dist_longitude,dist_latitude)) #Angulo del punto de destino
        #ang_rotacion = math.degrees(math.atan2(dist_latitude,dist_longitude)) #Angulo del punto de destino

        if ang_rotacion < 0:
            ang_rotacion = ang_rotacion + 360 #Convertir los angulos negativos

        self.heading = (ang_rotacion)  


        self.publish_velocity()      

        

def main():

    print("Nodo inicializado........")
    rospy.init_node('Navegacion')

    #Periodo de muestreo
    rospy.Rate(25)

    Navegacion = Node_navegation_drone()

    while not rospy.is_shutdown():
        
        Navegacion.goto()
        print(Navegacion.heading)




if __name__ == "__main__":
    main()

print("Script dead")