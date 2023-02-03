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
#compass = Ev3devSensor(Port.S2)
ultra = UltrasonicSensor(Port.S3)

#Vars
running = False #Je kamion v pohybu
canSteerLeft = True #Muze zatacet doleva
canSteerRight = True #Muze zatacet doprava 
stoppedLeft = False #Zataceni doleva bylo uz zastaveno
stoppedRight = False #Zataceni doprava bylo uz zastaveno
stoppedForward = False #Jizda vpred byla zastavena
canForward = True #Muze jet vpred
driveSpeed = 1560 #Rychlost motoru, v deg/s 
steerSpeed = 1560
steerAngle = 5 #O kolik stupnu se ma motor otocit pri jednom stisknuti tlacitka
maxAngle = 70 #Maximalni uhel na ktery se muze otocit motor ktery ovlada zataceni

#Main functions

#Funkce na kontrolu zataceni, kontroluje uhel zataceciho motoru, pokud prekroci maxAngle, zabrani zataceni
def steerCheck():  
    global stoppedLeft
    global stoppedRight
    global canSteerRight
    global canSteerLeft
    steer = steering.angle() #Uhel na motoru
    #Kontrola pro kladny uhel = natoceni kol doprava
    if steer >= maxAngle
        canSteerRight = False #Nemuze zatacet
        if (not stoppedRight): #Zastavi kola pouze jednou
            steering.brake()
            stoppedRight = True
    else:
        canSteerRight = True
        stoppedRight = False
    #To same pro zaporne hodnoty = natoceni kol doleva 
    if steer <= (-(maxAngle)):
        canSteerLeft = False
        if (not stoppedLeft):
            steering.brake()
            stoppedLeft = True
    else:
        canSteerLeft = True
        stoppedLeft = False
    #print(steer,canSteerR,canSteerL)


#Nouzove brzdeni pokud je prekazka pred kamionem
#Kontroluje vzdalenost v cm pred kamionem, pokud je mensi jak urcita hodnota provede akci
def EStop():
    global canForward
    global stoppedForward
    distance = ultra.distance() #Zmerena vzdalenost
    
    #Prekazka pred kamionem 
    if distance < 180:
        ev3.speaker.beep(750) #Vydava vysoky ton
        ev3.light.on(Color.RED) #Zacne sviti cervenou barvou
        canForward = False #Zamezi jizde v pred
        if not stoppedForward: #Zastavi kamion, pouze jednou
            drive.brake()
            stoppedForward = True
    
    
    #Prekazka nebezpecne blizko
    elif distance < 300:
        ev3.speaker.beep(600) #Vydava ton 
        ev3.light.on(Color.ORANGE) #Sviti oranzovou barvou
        canForward = True 
        stoppedForward = False
    
    #Prekazka v dalce
    elif distance < 400: 
        ev3.speaker.beep(600) #Vydava ton
        ev3.light.on(Color.GREEN) #Sviti zelene, vychozi barva
        canForward = True
        stoppedForward = False
    
    #Zadna prekazka, nastavi vychozi hodnoty
    else:
        ev3.light.on(Color.GREEN)
        canForward = True
        stoppedForward = False
    #print(distance,canForward)


#Hlavni cyklus
while True:
    buttons = infra.keypad() #Ziska stiknuta tlacitka
    #print(buttons) 
    
    #Kombinace tlacitek
    if len(buttons)>1:
        #Jizda vpred 
        if buttons[0] == Button.LEFT_UP and buttons[1] == Button.RIGHT_UP:
            if (not running) and canForward: #Pokud kamion uz jede, zastavi ho
                drive.run(speed = driveSpeed)
                running = True
            else:
                drive.stop()
                running = False
        #Jizda vzad
        if buttons[0] == Button.LEFT_DOWN and buttons[1] == Button.RIGHT_DOWN:
            if running:
                drive.stop()
                running = False
            else:
                drive.run(speed = -(driveSpeed))
                running = True 
    #Samostatna tlacitka
    elif len(buttons)==1:
        if buttons[0] == Button.LEFT_UP: #Zataceni doleva
            if canSteerLeft:
                steering.run_angle(speed = -(steerSpeed),rotation_angle = steerAngle, wait=False)
        if buttons[0] == Button.RIGHT_UP: #Zataceni doprava
            if canSteerRight:
                steering.run_angle(speed = steerSpeed, rotation_angle = steerAngle, wait=False)
        if buttons[0] == Button.LEFT_DOWN:
            ev3.speaker.beep()
            pass
        if buttons[0] == Button.RIGHT_DOWN:
            pass
    steerCheck()
    EStop()
    #print(readCompass())
