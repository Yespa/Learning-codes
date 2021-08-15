from dronekit import connect, VehicleMode, LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse


def connectMyCopter():
    parser = argparse.ArgumentParser(description='comands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 5760

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Esperando que el drone este disponible")
        print("")
    print("El dron esta listo para ser armado")

    vehicle.armed=True
    while vehicle.armed==False:
        print("Esperando que se arme el dron")
        time.sleep(1)
    print("El dron esta armado!")
    print("PRECAUCION CON LAS HELICES")

    return None

vehicle = connectMyCopter()
arm()
print("FIN")

