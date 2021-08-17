#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

#Librerias que permiten la comunicacion con la tarjeta de control de vuelo, Pixhawk 2.4.8

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil ## Libreria usada para definir los mensajes de comando

#Importaciones generales
import time
import socket
import math
import argparse

from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from std_msgs.msg import String


class Node_functions_drone:

    """
    Nodo que permite la interaccion con el drone
    
    Por medio de suscriptores de haran modificaciones en velocidades lineales, velocidades angulares en todos
    los ejes y mediante publicadores se enviaran datos de los sensores de la tarjeta controladora tales como
    GPS, datos de odometría, estado de la IMU, estado de dron...    
    """

    def __init__(self,connection_string,baud_rate):
        
        ##Realizar la conexion entre la Pixhawk y la Jetson nano
        self.connection_string = connection_string
        self.baud_rate = baud_rate

        self.vehicle = connect(self.connection_string,self.baud_rate,wait_ready=True)

        print("--------------CONEXION EXITOSA-------------------")

        rate = rospy.Rate(100)

        ##Subscriptores

        #sub_vel_lin = rospy.Subscriber('drone/vel_lin',String,METODO)

        #PUBLICADORES

        #Publicador de la coordenadas globales actuales de dron
        self.pub_pos_gps = rospy.Publisher('drone/pos_gps',NavSatFix,queue_size=10)

        #Publicador de la velocidad lineal actual del dron
        self.pub_vel_now = rospy.Publisher('drone/vel_now',TwistStamped,queue_size=10)

        #Publicador del angulo de orientacion del dron con respecto al norte de la tierra.
        self.pub_orient_z_now = rospy.Publisher('drone/orient_z_now', QuaternionStamped, queue_size=10)

        #Publicador del estado de la bateria actual del dron
        self.pub_battery_now = rospy.Publisher('drone/battery_now', BatteryState, queue_size=10)

        #Publicador de la actitud actual del dron
        self.pub_attitude_now = rospy.Publisher('drone/attitude_now',QuaternionStamped, queue_size=10)

    #METODOS


    #Metodo para obtener la posicion actual (latitud, longitud y altura)
    def publish_pos_gps(self):

        #Este mensaje tiene una estructura especial es por ellos que se formula de la siguiente manera
        msg =NavSatFix()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "drone/Data_GPS"
        msg.status.status = self.vehicle.gps_0.fix_type #Estado del GPS
        msg.status.service = self.vehicle.gps_0.satellites_visible #Cantidad de satelites conectados


        #Solicito posicion a la pixhawk
        msg.latitude =  self.vehicle.location.global_relative_frame.lat #Latitud
        msg.longitude =  self.vehicle.location.global_relative_frame.lon #Longitud
        msg.altitude =  self.vehicle.location.global_relative_frame.alt #Altura
        msg.position_covariance = (0,0,0,0,0,0,0,0,0) 
        msg.position_covariance_type = 0

        #Publico la informacion
        self.pub_pos_gps.publish(msg)

    #Metodo para publicar variables generales de los sensores del dron
    def publish_status_drone(self):

        #Se solicita la información a la pixhawk
        # --> Lleno los vectores de velocidad lineales y angulares
        
        vel_now = TwistStamped()

        vel_now.header.stamp = rospy.Time.now()
        vel_now.header.frame_id = "drone/Velocidades"
        vel_now.twist.linear.x = self.vehicle.velocity[0]
        vel_now.twist.linear.x = self.vehicle.velocity[1]
        vel_now.twist.linear.x = self.vehicle.velocity[2]
        vel_now.twist.angular.x = 0
        vel_now.twist.angular.y = 0
        vel_now.twist.angular.z = 0


        #Orientacion actual con respecto al eje z
        
        orientacion_z = QuaternionStamped()
        

        orientacion_z.header.stamp = rospy.Time.now()
        orientacion_z.header.frame_id = "drone/orientacion"
        orientacion_z.quaternion.x = 0
        orientacion_z.quaternion.y = 0
        orientacion_z.quaternion.z = self.vehicle.heading

        #Estado de la bateria del dron

        battery_now = BatteryState()

        battery_now.header.stamp = rospy.Time.now()
        battery_now.header.frame_id = "drone/battery"
        battery_now.voltage = self.vehicle.battery.voltage
        battery_now.current = self.vehicle.battery.current
        battery_now.percentage = self.vehicle.battery.level

        #Actitud actual del dron
        attitud_now = self.vehicle.attitude

                
        attitud_now = QuaternionStamped()
        

        attitud_now.header.stamp = rospy.Time.now()
        attitud_now.header.frame_id = "drone/attitud"
        attitud_now.quaternion.x = self.vehicle.attitude.roll
        attitud_now.quaternion.y = self.vehicle.attitude.pitch
        attitud_now.quaternion.z = self.vehicle.attitude.yaw


        self.pub_vel_now.publish(vel_now)
        self.pub_orient_z_now.publish(orientacion_z)
        self.pub_battery_now.publish(battery_now)
        self.pub_attitude_now.publish(attitud_now)


    #Metodo encargado de realizar el armado y despegue del dron

    def arm_takeoff(self,alt_deseada):

        self.vehicle.mode = VehicleMode("GUIDED")
           
        while self.vehicle.is_armable==False:
            print("Esperando disponibilidad de armado")
            print("")
            time.sleep(2)
        print("El dron esta listo para ser armado!")

        print("CUIDADO")
        time.sleep(3)

        self.vehicle.armed=True  ## Comando para armar el dron
        
        while self.vehicle.armed==False:
            print("Esperando que se arme el dron")
            time.sleep(1)
        print("------El dron esta armado!------")

        ######### Enviamos accion de despegue ##########
        time.sleep(5)
        print("------Despegando-------")
        self.vehicle.simple_takeoff(alt_deseada)        

        #Mostramos la altura actual y el codigo se detiene hasta que no llegue a la altura deseada, esto se hace
        #debido a que si se ejecuta otro comando, inmendiatamente se interrumpe la acción anterior.

        while True:
            print(" Altura actual: ", self.vehicle.location.global_relative_frame.alt)

            #Verificamos que no exceda la altura deseada, cuando se cumple salimos de la funcion

            if self.vehicle.location.global_relative_frame.alt >= alt_deseada * 0.95:
                print("")
                print("------Altura deseada alcanzada-----")
                break
            time.sleep(1)


def main():

    print("Nodo inicializado........")
    rospy.init_node('drone')
    drone = Node_functions_drone("127.0.0.1:14550",5760)    

    while not rospy.is_shutdown():
        
        if drone.vehicle.armed == False:
        
            drone.arm_takeoff(10)


        drone.publish_pos_gps()
        drone.publish_status_drone()

if __name__ == "__main__":
    main()

print("Script muerto")