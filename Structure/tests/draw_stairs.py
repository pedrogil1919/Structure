'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Example of a drawing of a number of stairs.
'''

import numpy
import cv2

from physics.stairs import Stair

stairs_list = (
    {'N': 2, 'd': 25.0, 'w': 50.0, 'h': +25.0},
    {'N': 3, 'd': 25.0, 'w': 50.0, 'h': -25.0},
    {'N': 2, 'd': 25.0, 'w': 50.0, 'h': +20.0}
    )
landing = 100.0

stair_test = Stair(stairs_list, landing)

size = (600, 1000)   # HD resolution.
origin = (0, 200)
image = numpy.full((size[0], size[1], 3), 0xFF, numpy.uint8)
scale = 15.0
shift = 3
stair_test.draw(origin, image, scale, shift)
cv2.imshow("img", image)
cv2.waitKey()

###############################################################################
# End of file.
###############################################################################