#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exceptions
import math
import argparse

#Primer codigo de prueba para el dron, consiste en armar el dron, despegar a 2 metros, mantenerse estable 30 segundos
#y posterior a ello aterrizar

#William Yesid Palencia Arismendy

############################### PRIMERO NOS CONECTAMOS A LA PIXHAWK ###################################
def connectMyCopter():
    parser = argparse.ArgumentParser(description='Comandos del drone')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 5760     ##Verificar ya que este valor varía dependiendo si estoy con la pixhawk o realizando la simulacion

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)

    print("Conexión exitosa")

    return vehicle

############################## ARMAMOS EL DRON Y REALIZAMOS EL DESPEGUE ####################################

def arm_despegue(alt_deseada):

    vehicle.mode = VehicleMode("GUIDED")  #Definimos el modo guiado para el despegue
    
    while vehicle.is_armable==False:
        print("Esperando disponibilidad de armado")
        print("")
    print("El dron esta listo para ser armado!")

    print("CUIDADO")
    time.sleep(3)

    vehicle.armed=True  ## Comando para armar el dron
    
    while vehicle.armed==False:
        print("Esperando que se arme el dron")
        time.sleep(1)
    print("El dron esta armado!")

    ######### Enviamos accion de despegue ##########
    time.sleep(2)
    print("-----Despegando-------")
    vehicle.simple_takeoff(alt_deseada)

    #Mostramos la altura actual y el codigo se detiene hasta que no llegue a la altura deseada, esto se hace
    #debido a que si se ejecuta otro comando, inmendiatamente se interrumpe la acción anterior.

    while True:
        print(" Altura actual: ", vehicle.location.global_relative_frame.alt)

        #Verificamos que no exceda la altura deseada, cuando se cumple salimos de la funcion

        if vehicle.location.global_relative_frame.alt >= alt_deseada * 0.95:
            print("Altitud deseada alcanzada")
            break
        time.sleep(1)


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
        time.sleep(4)
        time.sleep(1)




vehicle = connectMyCopter()
arm_despegue(5) # Defino el valor al cual quiero elevar el drone

print("Esperando aterrizaje")
time.sleep(20)

aterrizaje()


print("FIN DE LA MISION")


