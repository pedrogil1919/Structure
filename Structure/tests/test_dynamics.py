'''
Created on 16 feb. 2021

@author: diego
'''

import numpy
import cv2

from physics import stairs
from structure import base
from simulator.simulator import Simulator

image = numpy.full((500, 800, 3), 0xFF, numpy.uint8)

# Positive steps.
landing = 2000.0
stair_list = [
    {'N': 2, 'd': 250.0, 'w': 100.0, 'h': +40.0}
    ]
size = {
    'a': 40,
    'b': 80,
    'c': 30,
    'd': 50,
    'g': 50}
wheels = {
    'r1': 15,
    'r2': 15,
    'r3': 15,
    'r4': 15}
stair_test = stairs.Stair(stair_list, landing)                
base_test = base.Base(size, wheels, stair_test)

speed_data = {
    'wheel': 5.0,
    'actuator': 1.0,
    'struct_up': 0.5,
    'struct_down': 1.0}
sm = Simulator(speed_data)

image[:] = 0xFF
stair_test.draw((0, 400), image, 8, 3)
base_test.draw((0, 400), image, 8, 3)
cv2.imshow("image", image)
cv2.waitKey()

command = {'distance': 300}
for res in sm.simulate_instruction(base_test, command):
    image[:] = 0xFF
    stair_test.draw((0, 400), image, 8, 3)
    base_test.draw((0, 400), image, 8, 3)
    cv2.imshow("image", image)
    cv2.waitKey(5)
