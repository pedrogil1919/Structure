'''
Created on 9 jun. 2022

@author: pedrogil
'''
import unittest

import cv2
import numpy
from copy import deepcopy

from physics import stairs
from structure import base


class PushActuatorTest(unittest.TestCase):

    def draw(self, structure, stairs):
        image = numpy.full((600, 800, 3), 0xFF, numpy.uint8)
        origin = 600.0
        stairs.draw((40, origin), image, 4, 3)
        structure.draw((40, origin), image, 4, 3)
        cv2.imshow("res", image)
        cv2.waitKey()

    def test_push_actuator(self):
        # Check operation fo function make_room_wheel3 on base, when there is
        # not inclination limits (the structure can incline enough).
        landing = 500.0
        stair_list = [
            {'N': 5, 'd': 1000.0, 'w': 250.0, 'h': -80.0}
        ]
        size = {
            'a': 120.0,
            'b': 150.0,
            'c': 140.0,
            'd': 100.0,
            'h': 2.0,
            'v': 2.0,
            'g': 100.0,
            'n': 0.0}
        wheels = {
            'r1': 25.0,
            'r2': 25.0,
            'r3': 25.0,
            'r4': 25.0}
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion1t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion2t(struct_test)
        # self.draw(struct_test, stair)
        # To test this error, we need to make the step greater than above
        # (I do not know if this kind of error is possible in practice), and
        # set the maximum inclination to 0, only to make easier the computation
        # of the error values.
        stair_list = [
            {'N': 5, 'd': 1000.0, 'w': 250.0, 'h': -120.0}
        ]
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion3t(struct_test)
        # self.draw(struct_test, stair)
        # NOTE: To see a graphic representation of the structure end position
        # include this sentence wherever you want to see the position.
        # self.draw(struct_test, stair)

    def motion1t(self, structure):
        # Wheel collides with the stair, but the actuator does not.
        res = structure.elevate(10)
        self.assertTrue(res)
        res = structure.push_actuator(3, -10)
        self.assertTrue(res)
        res = structure.push_actuator(3, 60)
        self.assertFalse(res)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 0.0, 4)
        actuator = res.actuator(3)
        self.assertAlmostEqual(actuator, -50.0, 4)

    def motion2t(self, structure):
        # Wheel collides with the stair, and the actuator too, but the wheel
        # is limiting the motion.
        res = structure.advance(150)
        self.assertTrue(res)
        # res = structure.elevate(100)
        # self.assertTrue(res)
        res = structure.push_actuator(3, 110)
        self.assertFalse(res)
        actuator = res.actuator(3)
        self.assertAlmostEqual(actuator, -30.0, 4)

    def motion3t(self, structure):
        # Similar to 2, but now the actuator is limiting the motion.
        res = structure.advance(150)
        self.assertTrue(res)
        # res = structure.elevate(100)
        # self.assertTrue(res)
        res = structure.push_actuator(3, 130)
        self.assertFalse(res)
        actuator = res.actuator(3)
        self.assertAlmostEqual(actuator, -30.0, 4)
