#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
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

    print("Conexión exitosa")

    return vehicle


vehicle = connectMyCopter()

while True:
    
    time.sleep(5)


