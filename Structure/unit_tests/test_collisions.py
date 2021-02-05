'''
Created on 25 ene. 2021
@author: pedro.gil@uah.es
Test stairs constructor.
'''

import unittest

from physics import stairs, wheel


class CollisionTest(unittest.TestCase):

    def testConstructor(self):
        """Check if the wheel constructor raises correct exceptions.
        """
        # Positive steps.
        landing = 100.0
        stair_list = [
            {'N': 2, 'd': 250.0, 'w': 50.0, 'h': +25.0}
            ]
        radius = 15.0

        stairs_test = stairs.Stair(stair_list, landing)
        try:
            wheel.Wheel(radius, stairs_test, (125, 45))
        except ValueError:
            self.assertTrue(
                False, "Error: Wheel constructor raises exception.")
        self.assertRaises(
            ValueError, wheel.Wheel, radius, stairs_test, (125, 35))

        # Negative steps.
        stair_list = [
            {'N': 2, 'd': 250.0, 'w': 50.0, 'h': -25.0}
            ]
        radius = 15.0

        stairs_test = stairs.Stair(stair_list, landing)
        try:
            wheel.Wheel(radius, stairs_test, (125, 0))
        except ValueError:
            self.assertTrue(
                False, "Error: Wheel constructor raises exception.")
        self.assertRaises(
            ValueError, wheel.Wheel, radius, stairs_test, (125, -20))

    def testCollisionCorrection(self):
        """Check, when a wheel is in a forbidden position, if the wheel is moved
        to a correct one.
        """
        landing = 100.0
        stair_list = [
            {'N': 2, 'd': 250.0, 'w': 50.0, 'h': +25.0}
            ]
        radius = 15.0

        stairs_test = stairs.Stair(stair_list, landing)
        wheel_test = wheel.Wheel(radius, stairs_test, (radius, radius))
        position = [50, radius-10]
        res, w, h = wheel_test.move_wheel(position)
        self.assertFalse(res, "Error. Collision not detected.")
        position[0] += w
        position[1] += h
        res, w, h = wheel_test.move_wheel(position)
        self.assertTrue(res, "Error. Collision correction failed.")
        self.assertTrue(wheel_test.contact(),
                        "Error: Wheel not in contact after correction")
        
###############################################################################
# End of file.
###############################################################################