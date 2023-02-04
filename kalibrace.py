#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color

ev3 = EV3Brick()
light = ColorSensor(Port.S2)
max = 50
min = 50

while True:
    reflected = light.reflection()
    if reflected > max:
        max = reflected
    elif reflected < min:
        min = reflected
    if len(ev3.buttons()) > 0:
        break

with open("barvy.txt", "w") as file:
    file.write(str(min),",",str(max))