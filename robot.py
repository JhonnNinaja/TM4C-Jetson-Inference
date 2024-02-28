
from __future__ import print_function
import cv2 as cv
import argparse
import serial
from time import sleep, time
import time 
import jetson.inference
import jetson.utils

ser = serial.Serial('/dev/ttyACM0', 115200)
i=0
ser.timeout=0.3


def movimiento1():
	ser.write(b'UP')
	time.sleep(1)
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'LEFT')
	time.sleep(0.61)
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'UP')
	time.sleep(1)
	ser.write(b'STOP')
	time.sleep(2)

def movimiento3():
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'UP')
	time.sleep(1)
	ser.write(b'STOP')
	time.sleep(2)

def movimiento2():
	ser.write(b'DOWN')
	time.sleep(1)
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'RIGTH')
	time.sleep(0.63)
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'UP')
	time.sleep(3)
	ser.write(b'STOP')
	time.sleep(2)
	ser.write(b'LEFT')
	time.sleep(0.66)
		
camera = jetson.utils.videoSource("/dev/video0")
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold = 0.5)

movimiento1()
flag_11 = True

flag_person = 0
flag_cat = 0


while flag_11:
    img = camera.Capture()
    detections = net.Detect(img)
    for detection in detections:
        if(net.GetClassDesc(detection.ClassID) == "person"):
            print("Se detecta persona")
            flag_11 = False
            flag_person = 1
        if(net.GetClassDesc(detection.ClassID) == "cat"):
            print("Se detecta gato")
            flag_11 = False
            flag_cat = 1
        break
    time.sleep(1)
    
movimiento2()
movimiento3()
flag_22 = True
while flag_22:
    img = camera.Capture()
    detections = net.Detect(img)
    for detection in detections:
        if(net.GetClassDesc(detection.ClassID) == "person"):
            print("Se detecta persona")
            flag_22 = False
            flag_person = 1
        if(net.GetClassDesc(detection.ClassID) == "cat"):
            print("Se detecta gato")
            flag_22 = False
            flag_cat = 1
        break

    time.sleep(1)
print("se termino el programa")
ser.close()
