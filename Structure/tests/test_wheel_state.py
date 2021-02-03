'''
Created on 28 ene. 2021

@author: pedro

@author: pedro.gil@uah.es

Test wheel state functions with GUI. Use arrow keys to move the wheel and check
the correct color (state) of the wheel:
- 8: up
- 2: down
- 4: left
- 6: right
- 5: Place the wheel in a valid position when inside the stair.
- Esc: Finish the program.

'''

import numpy
import cv2

from physics import stairs, wheel

# Define a list of stairs in different directions.
landing = 100.0
stair_list = [
    {'N': 2, 'd': 60.0, 'w': 40.0, 'h': +25.0},
    {'N': 3, 'd': 40.0, 'w': 30.0, 'h': -20.0},
    {'N': 2, 'd': 60.0, 'w': 40.0, 'h': +25.0}
    ]
# Define wheel.
radius = 15.0
origin = (0, 250)
position = [15.0, 15.0]
scale = 16
shift = 3
# Create stairs.
stairs_test = stairs.Stair(stair_list, landing)
# Create wheel.
wheel_test = wheel.Wheel(radius, stairs_test, position)

image = numpy.full((600, 1000, 3), 0xFF, numpy.uint8)
while True:
    # Move the wheel, and get (in that case) the distance needed to correct the
    # position to place the wheel in a valid position (w, h).
    res, w, h = wheel_test.check_wheel(position)
    stairs_test.draw(origin, image, scale, shift)
    wheel_test.draw(origin, image, position, scale, shift)
    cv2.imshow("Res", image)
    c = cv2.waitKey()
    # Finish program.
    if c == 27:
        break
    elif c == ord('8'):
        # Up
        position[1] += 1
    elif c == ord('2'):
        # Down
        position[1] -= 1
    elif c == ord('6'):
        # Left
        position[0] += 1
    elif c == ord('4'):
        # Right
        position[0] -= 1
    elif c == ord('5'):
        # Place the wheel in a safe position.
        if not res:
            position[0] += w
            position[1] += h
    image[:] = 0xFF
    
###############################################################################
# End of file.
###############################################################################    