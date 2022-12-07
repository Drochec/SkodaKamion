#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import threading

#Sensors and Motors
ev3 = EV3Brick()
drive = Motor(Port.C)
steering = Motor(Port.B)
infra = InfraredSensor(Port.S1)
compass = Ev3devSensor(Port.S2)
ultra = UltrasonicSensor(Port.S3)

#Vars
running = False
canSteerL = True
canSteerR = True
canForward = True
maxAngle = 30
minAngel = -30


#Main functions
def steerCheck():
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
    distance = ultra.distance()
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
steerCheck = threading.Thread(target = steerCheck, daemon = True)
EStop = threading.Thread(target = EStop, daemon = True)

steerCheck.start()
EStop.start()
#Main Loop
while True:
    buttons = infra.keyboard()
    if len(buttons)>1:
        if isinstance(buttons[0], Button.Left_UP) and isinstance(buttons[1], Button.Right_UP):
            if (not running) and canForward:
                drive.run(50)
                running = True
            else:
                drive.stop()
                running = False
        if isinstance(buttons[0], Button.Left_DOWN) and isinstance(buttons[1], Button.Right_DOWN):
            if running:
                drive.stop()
                running = False
            else:
                drive.run(-50)
                running = True 
    else:
        if isinstance(buttons[0], Button.Left_UP):
            if canSteerL:
                steering.run_angle(5,5) 
        if isinstance(buttons[0], Button.Right_UP):
            if canSteerR:
                steering.run_angle(5,5)
        if isinstance(buttons[0], Button.Left_DOWN):
            ev3.beep()
        if isinstance(buttons[0], Button.Right_DOWN):
            pass

