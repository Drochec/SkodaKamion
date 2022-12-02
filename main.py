#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
infrared_1 = InfraredSensor(Port.S1)
pohon = Motor(Port.C)
zataceni = Motor(Port.B)
kompas = Ev3devSensor(Port.S2)

while True:
    tlac = infrared_1.buttons(1)
    if len(tlac) != 0:
        print(tlac)
    for i in tlac:
        print(i)
        if str(i) == "Button.LEFT_UP":
            zataceni.run_angle(10,20,Stop.BRAKE,False)
    
    
