from gpiozero import DistanceSensor
from time import sleep


sensorl = DistanceSensor(echo=25, trigger=24)
sensorr = DistanceSensor(echo=16, trigger=26)
while True:
    print("Distance", sensor.distance *100)
    drive.sendLeft(0.8)
    drive.sendRight(0.8)
    if (sensor.distance*100) < 20:
        drive.sendLeft(0)
        drive.sendRight(0)
        print("obstacle detected")
    sleep(1)

