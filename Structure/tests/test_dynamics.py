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
    'g': 50,
    'm': 1}
wheels = {
    'r1': 15,
    'r2': 15,
    'r3': 15,
    'r4': 15}
stair_test = stairs.Stair(stair_list, landing)
base_test = base.Base(size, wheels, stair_test)

speed_data = {
    'wheel': 2.0,
    'actuator_up': 2.0,
    'actuator_dw': 3.0,
    'elevate_up': 0.5,
    'elevate_dw': 1.0,
    'incline_up': 0.4,
    'incline_dw': 0.8,
    'sample_time': 1.0}

sm = Simulator(speed_data)

image[:] = 0xFF
stair_test.draw((0, 400), image, 8, 3)
base_test.draw((0, 400), image, 8, 3)
cv2.imshow("image", image)
cv2.waitKey()

command = {'distance': 200, 'elevate': 10, 'incline': 20}
y = 0
for res in sm.simulate_instruction(base_test, command):
    image[:] = 0xFF
    stair_test.draw((0, 400), image, 8, 3)
    base_test.draw((0, 400), image, 8, 3)
    cv2.imshow("image", image)
    cv2.waitKey(5)
    print(y)
    y += 1
    if y == 410:
        pass
