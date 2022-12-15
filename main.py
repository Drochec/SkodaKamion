#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

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
stoppedL = False
stoppedR = False
canForward = True
driveSpeed = 1560
steerSpeed = 1560
steerAngle = 5
maxAngle = 70

#Main functions
def steerCheck():
    global stoppedL
    global stoppedR
    global canSteerR
    global canSteerL
    steer = steering.angle()
    if steer >= maxAngle: #Kladne
        canSteerR = False
        if (not stoppedR):
            steering.brake()
            stoppedR = True
    else:
        canSteerR = True
        stoppedR = False
    if steer <= (-(maxAngle)): #Zaporne
        canSteerL = False
        if (not stoppedL):
            steering.brake()
            stoppedL = True
    else:
        canSteerL = True
        stoppedL = False
    #print(steer,canSteerR,canSteerL)

def EStop():
    global canForward
    distance = ultra.distance()
    if distance < 180:
        ev3.speaker.beep(750)
        ev3.light.on(Color.RED)
        canForward = False
        drive.brake()
    elif distance < 300:
        ev3.speaker.beep(600)
        ev3.light.on(Color.ORANGE)
        canForward = True
    elif distance < 400:
        ev3.speaker.beep()
        ev3.light.on(Color.GREEN)
        canForward = True
    else:
        canForward = True
    #print(distance,canForward)
    
def readCompass():
    return compass.read("COMPASS")

while True:
    buttons = infra.keypad()
    print(buttons)
    if len(buttons)>1:
        if buttons[0] == Button.LEFT_UP and buttons[1] == Button.RIGHT_UP:
            if (not running) and canForward:
                drive.run(speed = driveSpeed)
                running = True
            else:
                drive.stop()
                running = False
        if buttons[0] == Button.LEFT_DOWN and buttons[1] == Button.RIGHT_DOWN:
            if running:
                drive.stop()
                running = False
            else:
                drive.run(speed = -(driveSpeed))
                running = True 
    elif len(buttons)==1:
        if buttons[0] == Button.LEFT_UP:
            if canSteerL:
                steering.run_angle(speed = -(steerSpeed),rotation_angle = steerAngle, wait=False) 
        if buttons[0] == Button.RIGHT_UP:
            if canSteerR:
                steering.run_angle(speed = steerSpeed, rotation_angle = steerAngle, wait=False)
        if buttons[0] == Button.LEFT_DOWN:
            ev3.speaker.beep()
            pass
        if buttons[0] == Button.RIGHT_DOWN:
            pass
    steerCheck()
    EStop()
    print(readCompass())
