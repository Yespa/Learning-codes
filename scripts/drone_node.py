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

#from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
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

        rate = rospy.Rate(10)

        ##Subscriptores

        #sub_vel_lin = rospy.Subscriber('drone/vel_lin',String,METODO)

        #Publicadores

        self.pub_pos_gps = rospy.Publisher('drone/pos_gps',Point,queue_size=10)



    #Metodo para obtener la posicion actual (latitud, longitud y altura)
    def publish_pos_gps(self):


        while not rospy.is_shutdown():

            pos_act_gps_x =  self.vehicle.location.global_relative_frame.lat
            pos_act_gps_y =  self.vehicle.location.global_relative_frame.lon
            pos_act_gps_z =  self.vehicle.location.global_relative_frame.alt
            
            self.pub_pos_gps.publish(pos_act_gps_x,pos_act_gps_y,pos_act_gps_z)




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
    drone.arm_takeoff(10)
    drone.publish_pos_gps()

if __name__ == "__main__":
    main()

print("Script muerto")