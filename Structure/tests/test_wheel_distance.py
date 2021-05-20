'''
Created on 28 ene. 2021

@author: pedro

@author: pedro.gil@uah.es

Test the function distance.
Use arrow keys to move the wheel:
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
    {'N': 4, 'd': 60.0, 'w': 40.0, 'h': -25.0}
    ]
# Define wheel.
radius = 15.0
origin = (0, 160)
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
    res = wheel_test.get_distances(position)
    stairs_test.draw(origin, image, scale, shift)
    wheel_test.draw(origin, image, position, scale, shift)
    pos1 = (position[0] + res['wr'], position[1] + res['hr'])
    wheel_test.draw(origin, image, pos1, scale, shift)
    try:
        pos2 = (position[0], position[1] + res['hc'])
        wheel_test.draw(origin, image, pos2, scale, shift)
    except KeyError:
        pass
    try:
        pos3 = (position[0] + res['wc'], position[1] + res['hr'])
        wheel_test.draw(origin, image, pos3, scale, shift)
    except KeyError:
        pass
    cv2.imshow("Res", image)
    c = cv2.waitKey() & 0x7F
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
    image[:] = 0xFF
    
###############################################################################
# End of file.
###############################################################################    