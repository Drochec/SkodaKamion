#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
#from pybricks.iodevices import Ev3devSensor

#TODO
#Vypínání ovladačem
#Sledování čářy
#Sledování čáry PID


#Nastavení senzorů a motorů
ev3 = EV3Brick()
drive = Motor(Port.C)
steering = Motor(Port.B)
infra = InfraredSensor(Port.S1)
#compass = Ev3devSensor(Port.S2)
ultra = UltrasonicSensor(Port.S3)

#Proměnné
running = False #Je kamion v pohybu
canSteerLeft = True #Může zatáčet doleva
canSteerRight = True #Může zatáčet doprava 
stoppedLeft = False #Zatáčení doleva bylo už zastaveno
stoppedRight = False #Zatáčení doprava bylo už zastaveno
stoppedForward = False #Jízda vpřed byla zastavena
canForward = True #Může jet vpřed
driveSpeed = 1560 #Rychlost motoru, v stupních za sekundu (1560 ma)
steerSpeed = 1560
steerAngle = 5 #O kolik stupňů se má motor otočit při jednom stisknutí tlačítka
maxAngle = 70 #Maximálni úhel na který se může otočit motor který ovládá zatáčení
degreesToAvoid = 720 #Kolik stupňů musí kamion ujet dokud nebude mimo překážku


#Hlavní funkce

#Funkce na kontrolu zatáčení, kontroluje úhel zatáčecího motoru, pokud překročí maxAngle, zabrání zatáčení
def steerCheck():  
    global stoppedLeft
    global stoppedRight
    global canSteerRight
    global canSteerLeft
    steer = steering.angle() #Úhel na motoru
    
    #Kontrola pro kladný úhel = natočení kol doprava
    if steer >= maxAngle
        canSteerRight = False #Nemůže zatáčet
        if (not stoppedRight): #Zastaví kola pouze jednou
            steering.brake()
            stoppedRight = True
    else:
        canSteerRight = True
        stoppedRight = False
    
    #To samé pro záporné hodnoty = natočení kol doleva 
    if steer <= (-(maxAngle)):
        canSteerLeft = False
        if (not stoppedLeft):
            steering.brake()
            stoppedLeft = True
    else:
        canSteerLeft = True
        stoppedLeft = False
    #print(steer,canSteerR,canSteerL)


#Nouzove brždění pokud je překážka před kamionem
#Kontroluje vzdálenost v cm před kamionem, pokud je menší jak určitá hodnota provede akci
def EStop():
    global canForward
    global stoppedForward
    distance = ultra.distance() #Změřená vzdálenost
    
    #Překážka před kamionem 
    if distance < 180:
        ev3.speaker.beep(750) #Vydává vysoký tón
        ev3.light.on(Color.RED) #Začne svítit červenou barvou
        canForward = False #Zamezí jízdě vpřed
        if not stoppedForward: #Zastaví kamion, pouze jednou
            drive.brake()
            stoppedForward = True
    
    
    #Překážka nebezpečně blízko
    elif distance < 300:
        ev3.speaker.beep(600) #Vydává tón 
        ev3.light.on(Color.ORANGE) #Svítí oranžovou barvou
        canForward = True 
        stoppedForward = False
    
    #Prekazka v dalce
    elif distance < 400: 
        ev3.speaker.beep(600) #Vydává tón
        ev3.light.on(Color.GREEN) #Svítí zeleně, výchozi barva
        canForward = True
        stoppedForward = False
    
    #Žádná překážka, nastaví výchozí hodnoty
    else:
        ev3.light.on(Color.GREEN)
        canForward = True
        stoppedForward = False
    #print(distance,canForward)

#Semiautonomní režim
def semiauto():
    turning = False
    while True:
        buttons = infra.keypad() #Získá stiknutá tlačítka
        if len(buttons)>0: #Při stiknutí jakéhokoliv tlačítka vypne režim
            break
        
        distance = ultra.distance() #Získá vzdálenost před kamionem
        drive.run(speed = driveSpeed) 
        if distance <= 180: #Jestli je před kamionem překážka zatočí
            drive.reset_angle()
            steering.run_angle(-70)
            turning = True
        elif turning and (drive.angle() >= degreesToAvoid):
            steering.run_angle(0)
            turning = False
            


#-------------
#Hlavní cyklus
#-------------
while True:
    
    #Kontroly
    EStop()
    steerCheck()
    

    buttons = infra.keypad() #Získá stisknutá tlačítka
    #print(buttons) 
    
    #Vypnutí programu
    if len(buttons)==4:
        steering.run_to_angle(0,wait=True)
        exit()
    
    
    #Kombinace tlačítek
    elif len(buttons)>1:
        #Jízda vpřed
        if buttons[0] == Button.LEFT_UP and buttons[1] == Button.RIGHT_UP:
            if canForward:
                drive.run(speed = driveSpeed)
                running = True
        #Jízda vzad
        if buttons[0] == Button.LEFT_DOWN and buttons[1] == Button.RIGHT_DOWN:
            drive.run(speed = -(driveSpeed))
            running = True 
    #Samostatná tlačítka
    elif len(buttons)==1:
        if buttons[0] == Button.LEFT_UP: #Zatáčení doleva
            if canSteerLeft:
                steering.run_angle(speed = -(steerSpeed),rotation_angle = steerAngle, wait=False)
        if buttons[0] == Button.RIGHT_UP: #Zatáčení doprava
            if canSteerRight:
                steering.run_angle(speed = steerSpeed, rotation_angle = steerAngle, wait=False)
        
        if buttons[0] == Button.LEFT_DOWN: #Brzda
            drive.brake()
            running = False
        
        #Zatím nevyužité
        if buttons[0] == Button.RIGHT_DOWN:
            pass
