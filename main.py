#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

import threading

#Sensors and Motors
ev3 = EV3Brick()
drive = Motor(Port.C)
steering = Motor(Port.B)
infra = InfraredSensor(Port.S1)
compass = Ev3devSensor(Port.S2)
#ultra = UltrasonicSensor(Port.S3)

#Vars
running = False
canSteerL = True
canSteerR = True
canForward = True
driveSpeed = 50
steerSpeed = 100
steerAngle = 10
maxAngle = 30
minAngel = -30

#Main functions
def steerCheck():
    while True:
        steer = steering.angle()
        if steer >= 30:
            canSteerL = False
            steering.brake()
        else:
            canSteerL = True
        if steer <= -30:
            canSteerR = False
            steering.brake()
        else:
            canSteerR = True

def EStop():
    while True:
    #distance = ultra.distance()
        distance = 200
        if distance < 30:
            ev3.speaker.beep()
            ev3.light.on(Color.GREEN)
            canDrive = True
        elif distance < 20:
            ev3.speaker.beep(600)
            ev3.light.on(Color.ORANGE)
            canDrive = True
        elif distance < 10:
            ev3.speaker.beep(750)
            ev3.light.on(Color.RED)
            canDrive = False
            drive.brake()

#Thread execution
steerCheck = threading.Thread(target = steerCheck)
EStop = threading.Thread(target = EStop)

#steerCheck.start()
#EStop.start()
#Main Loop
while True:
    buttons = infra.keypad()
    print(buttons)
    if len(buttons)>1:
        if buttons[0] == Button.LEFT_UP and buttons[1] == Button.RIGHT_UP:
            if (not running) and canForward:
                drive.run(driveSpeed)
                running = True
            else:
                drive.stop()
                running = False
        if buttons[0] == Button.LEFT_DOWN and buttons[1] == Button.RIGHT_DOWN:
            if running:
                drive.stop()
                running = False
            else:
                drive.run(-(driveSpeed))
                running = True 
    elif len(buttons)==1:
        if buttons[0] == Button.LEFT_UP:
            if canSteerL:
                steering.run_angle(steerAngle,steerSpeed) 
        if buttons[0] == Button.RIGHT_UP:
            if canSteerR:
                steering.run_angle(steerAngle,-(steerSpeed))
        if buttons[0] == Button.LEFT_DOWN:
            ev3.beep()
        if buttons[0] == Button.RIGHT_DOWN:
            pass

