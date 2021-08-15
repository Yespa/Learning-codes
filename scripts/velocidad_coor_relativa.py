#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil ## Libreria usada para definir los mensajes de comando
import time
import socket
import exceptions
import math
import argparse

#Segundo código de prueba, en este scrip se evalua el comportamiento del dron desplazandose en X, Y y Z

#William Yesid Palencia Arismendy



############################### PRIMERO NOS CONECTAMOS A LA PIXHAWK ###################################

def connectMyCopter():
    parser = argparse.ArgumentParser(description='Comandos del drone')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 5760     ##Verificar ya que este valor varía dependiendo si estoy con la pixhawk o realizando la simulacion

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)

    print("Conexión existosa")

    return vehicle

############################## Velocidades del dron ####################################

## Recordemos que los mensajes de la funciones hay que enviarlos cada segundo, si no es así,
## el dron se detiene


###### FUNCION PARA DEFINIR VELOCIDADES RESPECTO A CADA EJE ###############################


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
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
        vehicle.send_mavlink(msg)
        time.sleep(1)


########################## FUNCION USADA PARA REALIZAR MOVIMIENTOS EN YAW ####################


def condition_yaw(heading, relative=False):

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    #Se envia el comando al dron 
    vehicle.send_mavlink(msg)

    
    while True:
      
        print(" Cabeceo actual: ", vehicle.heading)

        #Nos metemos en un ciclo hasta que se cumpla el valor de ultima posicion del cabeceo
        #para que la funcion no sea interrumpida por otro comando

        if vehicle.heading > heading:
            if vehicle.heading <= heading * 0.95:
                print("------Orientacion deseada alcanzada-----")
                break
            time.sleep(1)
        else:
            if vehicle.heading >= heading * 0.95:
                print("------Orientacion deseada alcanzada-----")
                break
            time.sleep(1)



def adelante(VelMax):


    def sen(grados):
        seno = math.sin(math.radians(grados))
        return seno

    def cos(grados):
        coseno = math.cos(math.radians(grados))
        return coseno

    grados_act = vehicle.heading
    Velx=0
    Vely=0
   
    while (((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)) or (grados_act<= 100 and grados_act >=85) or (grados_act <= 185 and grados_act>=175) or (grados_act <= 275 and grados_act >=265)):
        

        if ((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)):
            Velx= VelMax
            Vely= 0
            break
        elif (grados_act <= 100 and grados_act>=85):
            Velx= 0
            Vely= VelMax
            break
        elif (grados_act <= 185 and grados_act >=175):
            Velx= -VelMax
            Vely= 0
            break
        elif (grados_act<= 275 and grados_act >=265):
            Velx= 0
            Vely= -VelMax
            break

    while (grados_act > 5 and grados_act <85) or (grados_act > 95 and grados_act <175) or (grados_act > 185 and grados_act <265) or (grados_act > 275 and grados_act< 355):
        

        if (grados_act> 5 and grados_act <355):
            Velx= VelMax*cos(grados_act)
            Vely= VelMax*sen(grados_act)
            break

    return send_ned_velocity(Velx,Vely,0,2)


def atras(VelMax):


    def sen(grados):
        seno = math.sin(math.radians(grados))
        return seno

    def cos(grados):
        coseno = math.cos(math.radians(grados))
        return coseno

    grados_act = vehicle.heading
    Velx=0
    Vely=0
   
    while (((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)) or (grados_act<= 100 and grados_act >=85) or (grados_act <= 185 and grados_act>=175) or (grados_act <= 275 and grados_act >=265)):
        

        if ((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)):
            Velx= VelMax
            Vely= 0
            break
        elif (grados_act <= 100 and grados_act>=85):
            Velx= 0
            Vely= VelMax
            break
        elif (grados_act <= 185 and grados_act >=175):
            Velx= -VelMax
            Vely= 0
            break
        elif (grados_act<= 275 and grados_act >=265):
            Velx= 0
            Vely= -VelMax
            break

    while (grados_act > 5 and grados_act <85) or (grados_act > 95 and grados_act <175) or (grados_act > 185 and grados_act <265) or (grados_act > 275 and grados_act< 355):
        

        if (grados_act> 5 and grados_act <355):
            Velx= VelMax*cos(grados_act)
            Vely= VelMax*sen(grados_act)
            break

    return send_ned_velocity(-Velx,-Vely,0,2)


def izquierda(VelMax):


    def sen(grados):
        seno = math.sin(math.radians(grados))
        return seno

    def cos(grados):
        coseno = math.cos(math.radians(grados))
        return coseno

    grados_act = vehicle.heading
    Velx=0
    Vely=0
   
    while (((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)) or (grados_act<= 100 and grados_act >=85) or (grados_act <= 185 and grados_act>=175) or (grados_act <= 275 and grados_act >=265)):
        

        if ((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)):
            Velx= VelMax
            Vely= 0
            break
        elif (grados_act <= 100 and grados_act>=85):
            Velx= 0
            Vely= VelMax
            break
        elif (grados_act <= 185 and grados_act >=175):
            Velx= -VelMax
            Vely= 0
            break
        elif (grados_act<= 275 and grados_act >=265):
            Velx= 0
            Vely= -VelMax
            break

    while (grados_act > 5 and grados_act <85) or (grados_act > 95 and grados_act <175) or (grados_act > 185 and grados_act <265) or (grados_act > 275 and grados_act< 355):
        

        if (grados_act> 5 and grados_act <355):
            Velx= VelMax*cos(grados_act)
            Vely= VelMax*sen(grados_act)
            break

    return send_ned_velocity(Velx,-Vely,0,2)

def derecha(VelMax):


    def sen(grados):
        seno = math.sin(math.radians(grados))
        return seno

    def cos(grados):
        coseno = math.cos(math.radians(grados))
        return coseno

    grados_act = vehicle.heading
    Velx=0
    Vely=0
   
    while (((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)) or (grados_act<= 100 and grados_act >=85) or (grados_act <= 185 and grados_act>=175) or (grados_act <= 275 and grados_act >=265)):
        

        if ((grados_act <= 5 and grados_act >=0) or (grados_act>=355 and grados_act <=360)):
            Velx= VelMax
            Vely= 0
            break
        elif (grados_act <= 100 and grados_act>=85):
            Velx= 0
            Vely= VelMax
            break
        elif (grados_act <= 185 and grados_act >=175):
            Velx= -VelMax
            Vely= 0
            break
        elif (grados_act<= 275 and grados_act >=265):
            Velx= 0
            Vely= -VelMax
            break

    while (grados_act > 5 and grados_act <85) or (grados_act > 95 and grados_act <175) or (grados_act > 185 and grados_act <265) or (grados_act > 275 and grados_act< 355):
        

        if (grados_act> 5 and grados_act <355):
            Velx= VelMax*cos(grados_act)
            Vely= VelMax*sen(grados_act)
            break

    return send_ned_velocity(-Velx,Vely,0,2)


     

        
vehicle = connectMyCopter()

print("Desplazamiento en X")

send_ned_velocity(-1,0,0,1)

time.sleep(5)

print("Rotación en Yaw")

condition_yaw(270)

time.sleep(5)

print("Adelante")

adelante(1)

time.sleep(5)

print("Atras")

atras(1)

time.sleep(5)

print("Izquierda")

izquierda(1)

time.sleep(5)

print("Derecha")


derecha(1)


print("FIN DE LA MISION")

