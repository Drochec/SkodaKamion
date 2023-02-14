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
#Vypínání ovladačem - h
#Sledování čářy - h
#Sledování čáry PID - h


#Nastavení senzorů a motorů
ev3 = EV3Brick()
drive = Motor(Port.C)
steering = Motor(Port.B)
infra = InfraredSensor(Port.S3)
#compass = Ev3devSensor(Port.S2)
ultra = UltrasonicSensor(Port.S1)
light = ColorSensor(Port.S2)
panto = Motor(Port.D)

#Proměnné
running = False #Je kamion v pohybu
canSteerLeft = True #Může zatáčet doleva
canSteerRight = True #Může zatáčet doprava
canForward = True #Může jet vpřed

stoppedLeft = False #Zatáčení doleva bylo už zastaveno
stoppedRight = False #Zatáčení doprava bylo už zastaveno
stoppedForward = False #Jízda vpřed byla zastavena

driveSpeed = 1560 #Rychlost motoru, v stupních za sekundu (1560 ma)
steerSpeed = 1560
lineSpeed = 260 #Rychlost při sledování čáry
steerAngle = 5 #O kolik stupňů se má motor otočit při jednom stisknutí tlačítka
maxAngle = 70 #Maximálni úhel na který se může otočit motor který ovládá zatáčení
degreesToAvoid = 720 #Kolik stupňů musí kamion ujet dokud nebude mimo překážku

black = 12 #Naměřená hodnota černé
white = 45 #Naměřená hodnota bílé
boundary = (black + white) / 2 #Rozmezí bílé a černé

#Hlavní funkce

#Funkce na kontrolu zatáčení, kontroluje úhel zatáčecího motoru, pokud překročí maxAngle, zabrání zatáčení
def steerCheck():  
    global stoppedLeft
    global stoppedRight
    global canSteerRight
    global canSteerLeft
    steer = steering.angle() #Úhel na motoru
    
    #Kontrola pro kladný úhel = natočení kol doprava
    if steer >= maxAngle:
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
#Kontroluje vzdálenost v mm před kamionem, pokud je menší jak určitá hodnota provede akci
def EStop():
    global canForward
    global stoppedForward
    distance = ultra.distance() #Změřená vzdálenost
    
    #Překážka před kamionem 
    if distance < 200:
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
    #elif distance < 400: 
    #    ev3.speaker.beep(600) #Vydává tón
    #    ev3.light.on(Color.GREEN) #Svítí zeleně, výchozi barva
    #    canForward = True
    #    stoppedForward = False
    
    #Žádná překážka, nastaví výchozí hodnoty
    else:
        ev3.light.on(Color.GREEN)
        canForward = True
        stoppedForward = False
    #print(distance,canForward)

#Semiautonomní režim
def semiauto():
    drive.run(driveSpeed)
    panto.run_target(lineSpeed, 310, wait=False)
    while True:
        buttons = infra.keypad() #Získá stiknutá tlačítka
        if len(buttons)>0: #Při stiknutí jakéhokoliv tlačítka vypne režim
            drive.brake()
            panto.run_target(lineSpeed, 5, wait=True)
            break

        
        distance = ultra.distance() #Získá vzdálenost před kamionem
        drive.run(speed = driveSpeed) 
        if distance <= 180: #Jestli je před kamionem překážka zatočí
            drive.brake()
            drive.reset_angle(0)
            drive.run_target(driveSpeed,degreesToAvoid)
            steering.run_target(steerSpeed,-70)
            drive.run_target(driveSpeed,-degreesToAvoid)
            steering.run_target(steerSpeed,0,wait=True)
            
        #print("Semi")
            
#Sledování čáry
def linefollower():
    drive.brake()
    panto.run_target(lineSpeed,310,wait=False)
    steering.run_target(steerSpeed,0,wait=True)
    drive.run(lineSpeed)
    while True:
        buttons = infra.keypad() #Získá stiknutá tlačítka
        if len(buttons)>0: #Při stiknutí jakéhokoliv tlačítka vypne režim
            drive.brake()
            steering.run_target(steerSpeed,0,wait=True)
            panto.run_target(lineSpeed,5,wait=True)
            break
        
        if light.reflection() > boundary:
            steering.run_target(steerSpeed,45,wait=False)
        else:
            steering.run_target(steerSpeed,-45,wait=False)
        #print("Line",light.reflection())
        
def linepd(bila,cerna):
    drive.brake() #Zastaví
    panto.run_target(lineSpeed,310,wait=False) #Vysune pantograf
    steering.run_target(steerSpeed,0,wait=True) #Zarovná kola

    hranice = (bila + cerna)/2 #Rozhraní černé a bílé
    
    #Konstanty a proměnné pro PID
    error = 0
    last_error = 0
    integral = 0
    derivative = 0
    kp = 1
    ki = 0.001 
    kd = 0.1

    while True:
        buttons = infra.keypad() #Získá stiknutá tlačítka
        if len(buttons)>0: #Při stiknutí jakéhokoliv tlačítka vypne režim
            steering.run_target(steerSpeed,0,wait=True) #Zarovná kola
            panto.run_target(lineSpeed,5,wait=True) #Zasune pantograf
            break

        error = light.reflection() - hranice #Proporcionální - aktuální odchylka
        integral += error #Integrační - sčítá chyby, snaží se předpovědět budoucí
        derivative = last_error - error #Derivační - podle předchozí chyby 
        result = kp*error + ki*integral + kd*derivative #Součet všeho
        if result > 70: #Proti přetočení kol
            result = 70
        elif result < -70:
            result = -70
        steering.run_target(steerSpeed,result,wait=False) #Zatočení
        last_error = error #Zápis chyby

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
        drive.brake()
        panto.run_target(lineSpeed,0,wait=False)
        steering.brake()
        steering.run_target(steerSpeed,0,wait=True)
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
        #Semiauto režim
        if buttons[0] == Button.RIGHT_UP and buttons[1] == Button.RIGHT_DOWN:
            semiauto()
        #Linefollower
        if buttons[0] == Button.LEFT_UP and buttons[1] == Button.RIGHT_DOWN:
            linepd(12,45)
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
            #linefollower()
            #semiauto()
            pass
    print(buttons)
