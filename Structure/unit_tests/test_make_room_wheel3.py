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


class MakeRoomWheelTest(unittest.TestCase):

    def draw(self, structure, stairs):
        image = numpy.full((600, 800, 3), 0xFF, numpy.uint8)
        origin = 600.0
        stairs.draw((40, origin), image, 4, 3)
        structure.draw((40, origin), image, 4, 3)
        cv2.imshow("res", image)
        cv2.waitKey()

    def testMakeRoomWheel3_no_inclination_limit(self):
        # Positive steps.
        landing = 500.0
        stair_list = [
            {'N': 5, 'd': 1000.0, 'w': 150.0, 'h': +87.5}
        ]
        size = {
            'a': 120.0,
            'b': 150.0,
            'c': 140.0,
            'd': 100.0,
            'h': 2.0,
            'v': 2.0,
            'g': 100.0,
            'n': 800.0}
        wheels = {
            'r1': 25.0,
            'r2': 25.0,
            'r3': 25.0,
            'r4': 25.0}
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion1(struct_test)
        struct_test = deepcopy(structure)
        self.motion2(struct_test)
        struct_test = deepcopy(structure)
        self.motion3(struct_test)
        self.draw(struct_test, stair)

    def motion1(self, structure):
        # Sin ningún tipo de colisión.
        res = structure.push_actuator(3, -60)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60, 4)

    def motion2(self, structure):
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -10)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 29.2857, 4)

    def motion3(self, structure):
        # Sin ningún tipo de colisión.
        res = structure.elevate(20)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 151.85185, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 48.1481, 4)

        print("Fin")
        # self.assertAlmostEqual(inclination, 100, 1e-3)
