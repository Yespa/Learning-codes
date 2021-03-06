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
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from std_msgs.msg import String


class Node_functions_drone:

    """
    Nodo que permite la interaccion con el drone
    
    Por medio de suscriptores de haran modificaciones en velocidades lineales, velocidades angulares en todos
    los ejes y mediante publicadores se enviaran datos de los sensores de la tarjeta controladora tales como
    GPS, datos de odometría, estado de la IMU, estado de dron...    
    """

    def __init__(self,connection_string,baud_rate):

        self.Vx = 0
        self.Vy = 0
        self.Vz = 0
        self.heading = 0
        
        ##Realizar la conexion entre la Pixhawk y la Jetson nano
        self.connection_string = connection_string
        self.baud_rate = baud_rate

        self.vehicle = connect(self.connection_string,self.baud_rate,wait_ready=True)

        print("--------------CONEXION EXITOSA-------------------")


        ##SUBSCRIPTORES

        self.sub_vel_nav = rospy.Subscriber('Nav/vel_lin',TwistStamped,self.update_velocity)

        #PUBLICADORES

        #Publicador de la coordenadas globales actuales de dron
        self.pub_pos_gps = rospy.Publisher('drone/pos_gps',NavSatFix,queue_size=10)

        #Publicador de la velocidad lineal actual del dron
        self.pub_vel_now = rospy.Publisher('drone/vel_now',TwistStamped,queue_size=10)

        #Publicador del angulo de orientacion del dron con respecto al norte de la tierra.
        self.pub_orient_angle_z_now = rospy.Publisher('drone/orient_angle_z_now', PointStamped, queue_size=10)

        #Publicador del estado de la bateria actual del dron
        self.pub_battery_now = rospy.Publisher('drone/battery_now', BatteryState, queue_size=10)

        #Publicador de la actitud actual del dron
        self.pub_orientacion_quaternion = rospy.Publisher('drone/orient_quaternion',QuaternionStamped, queue_size=10)

    #METODOS

    #Metodo para actualizar la velocidad entregada por el nodo de navegación

    def update_velocity(self,Vel_nav):
        self.Vx = Vel_nav.twist.linear.x
        self.Vy = Vel_nav.twist.linear.y
        self.Vz = Vel_nav.twist.linear.z
        self.heading = Vel_nav.twist.angular.z


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
        # --> VELOCIDADES ACTUALES EL DRON
        
        vel_now = TwistStamped()

        vel_now.header.stamp = rospy.Time.now()
        vel_now.header.frame_id = "drone/Velocidades"
        vel_now.twist.linear.x = self.vehicle.velocity[0]
        vel_now.twist.linear.y = self.vehicle.velocity[1]
        vel_now.twist.linear.z = self.vehicle.velocity[2]
        vel_now.twist.angular.x = 0
        vel_now.twist.angular.y = 0
        vel_now.twist.angular.z = 0


        #--> ANGULO EN GRADOS DE LA ROTACION EN EL EJE Z (Respecto al norte)
        
        point_ang_z = PointStamped()
        
        point_ang_z.header.stamp = rospy.Time.now()
        point_ang_z.header.frame_id = "drone/angle_z"
        point_ang_z.point.x = 0
        point_ang_z.point.y = 0
        point_ang_z.point.z = self.vehicle.heading

        #--> ESTADO DE LA BATERIA DEL DRON

        battery_now = BatteryState()

        battery_now.header.stamp = rospy.Time.now()
        battery_now.header.frame_id = "drone/battery"
        battery_now.voltage = self.vehicle.battery.voltage
        battery_now.current = self.vehicle.battery.current
        battery_now.percentage = self.vehicle.battery.level


        #--> ACTITUD O ORIENTACION ACTUAL DE DRON

        self.phi = self.vehicle.attitude.roll
        self.theta = self.vehicle.attitude.pitch
        self.psi = self.vehicle.attitude.yaw

        # Convertimos los angulos fijos, angulos de euler, a Quaternion para posteriormente publicar la orientacion
                #Conversion de tomada de Meccanimo Complesso https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
        self.qw = math.cos(self.phi/2) * math.cos(self.theta/2) * math.cos(self.psi/2) + math.sin(self.phi/2) * math.sin(self.theta/2) * math.sin(self.psi/2)
        self.qx = math.sin(self.phi/2) * math.cos(self.theta/2) * math.cos(self.psi/2) - math.cos(self.phi/2) * math.sin(self.theta/2) * math.sin(self.psi/2)
        self.qy = math.cos(self.phi/2) * math.sin(self.theta/2) * math.cos(self.psi/2) + math.sin(self.phi/2) * math.cos(self.theta/2) * math.sin(self.psi/2)
        self.qz = math.cos(self.phi/2) * math.cos(self.theta/2) * math.sin(self.psi/2) - math.sin(self.phi/2) * math.sin(self.theta/2) * math.cos(self.psi/2)
               
        attitud_now = QuaternionStamped()
        
        attitud_now.header.stamp = rospy.Time.now()
        attitud_now.header.frame_id = "drone/Orientacion_quaternion"

        attitud_now.quaternion.x = self.qx
        attitud_now.quaternion.y = self.qy
        attitud_now.quaternion.z = self.qz
        attitud_now.quaternion.w = self.qw

        self.pub_vel_now.publish(vel_now)
        self.pub_orient_angle_z_now.publish(point_ang_z)
        self.pub_battery_now.publish(battery_now)
        self.pub_orientacion_quaternion.publish(attitud_now)


    #METODO PARA REALIZAR EL ARMADO DE DRON

    def arm_takeoff(self):

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

    #METODO PARA REALIZAR EL DESPEGUE
    
    def takeoff(self,alt_deseada):
        
        if self.vehicle.armed == True:

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

        else:
            self.msg ='Dronenoarmed'
            print("drone no armado")


    #Metodo para enviar comandos de velocidad a la controladora de vuelo en coordenadas absolutas

                #Script predeterminado de la libreria de Dronekit. Tomado de la API de dronekit

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # Se envia el comando al dron en un ciclo de 1 Hz
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)


    #Metodo para convertir las velocidades con coordenadas realtivas a velocidad con coordenadas absolutas

    def Vel_mat_rot_Z(self):

        grados_act = self.vehicle.heading

        def sen(grados):
            seno = math.sin(math.radians(grados))
            return seno

        def cos(grados):
            coseno = math.cos(math.radians(grados))
            return coseno

        self.VxP = self.Vx*cos(grados_act) - self.Vy*sen(grados_act)
        self.VyP = self.Vx*sen(grados_act) + self.Vy*cos(grados_act)
        self.VzP = self.Vz

        self.send_ned_velocity(self.VxP,self.VyP,self.VzP,5)


    #Metodo para realizar la rotacion del dron en el eje Z

    def condition_yaw(self, relative=False):

        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            self.heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        
        #Se envia el comando al dron 
        self.vehicle.send_mavlink(msg)

        
        while True:
        
            print(" Cabeceo actual: ", self.vehicle.heading)

            #Nos metemos en un ciclo hasta que se cumpla el valor de ultima posicion del cabeceo
            #para que la funcion no sea interrumpida por otro comando

            if self.vehicle.heading > self.heading:
                if self.vehicle.heading <= self.heading * 0.95:
                    print("------Orientacion deseada alcanzada-----")
                    break
                time.sleep(1)
            else:
                if self.vehicle.heading >= self.heading * 0.95:
                    print("------Orientacion deseada alcanzada-----")
                    break
                time.sleep(1)


    #Metodo para realizar un aterrizaje controlado.

    def aterrizaje(self):
        #Cambiamos al modo aterrizaje
        
        self.vehicle.mode = VehicleMode("LAND")

        print("Aterrizando")

        while True:
            print(" Altura actual: ", self.vehicle.location.global_relative_frame.alt)

            #Verificamos que se mantenga exceda la altura deseada, cuando se cumple salimos de la funcion

            if self.vehicle.location.global_relative_frame.alt <= 1 * 0.95:
                print("--------Aterrizando-------")
                break
            time.sleep(1)

def main():

    print("Nodo inicializado........")
    rospy.init_node('drone')

    drone = Node_functions_drone("127.0.0.1:14550",5760) 

    #Periodo de muestreo
    rospy.Rate(25)


    while not rospy.is_shutdown():
        
        if drone.vehicle.armed == False:
        
            drone.arm_takeoff(10)


        drone.publish_pos_gps()

        #drone.condition_yaw()

        drone.publish_status_drone()

        drone.Vel_mat_rot_Z()


if __name__ == "__main__":
    main()

print("Script dead")