#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exceptions
import math
import argparse

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

############################## ATERRIZAMOS EL DRON ####################################

def aterrizaje():
    #Cambiamos al modo aterrizaje
    
    vehicle.mode = VehicleMode("LAND")

    print("Aterrizando")

    while True:
        print(" Altura actual: ", vehicle.location.global_relative_frame.alt)

        #Verificamos que se mantenga exceda la altura deseada, cuando se cumple salimos de la funcion

        if vehicle.location.global_relative_frame.alt <= 1 * 0.95:
            print("----Aterrizando---")
            break
        time.sleep(1)

vehicle = connectMyCopter()

aterrizaje()


print("FIN DE LA MISION")

