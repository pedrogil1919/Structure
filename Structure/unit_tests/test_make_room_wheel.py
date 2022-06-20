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

    # def testMakeRoomWheelN_no_inclination_limit(self):
    #     # Check operation fo function make_room_wheelN on base, when there is
    #     # not inclination limits (the structure can incline enough).
    #     landing = 500.0
    #     stair_list = [
    #         {'N': 5, 'd': 1000.0, 'w': 250.0, 'h': 80.0}
    #     ]
    #     size = {
    #         'a': 120.0,
    #         'b': 150.0,
    #         'c': 140.0,
    #         'd': 100.0,
    #         'h': 2.0,
    #         'v': 2.0,
    #         'g': 100.0,
    #         'n': 800.0}
    #     wheels = {
    #         'r1': 25.0,
    #         'r2': 25.0,
    #         'r3': 25.0,
    #         'r4': 25.0}
    #     stair = stairs.Stair(stair_list, landing)
    #     structure = base.Base(size, wheels, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion1tN(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion2tN(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion3t(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion4t(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion5t(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion6t(struct_test)
    #     self.draw(struct_test, stair)
    #     struct_test = deepcopy(structure)
    #     self.motion7t(struct_test)
    #     self.draw(struct_test, stair)
    #     # NOTE: To see a graphic representation of the structure end position
    #     # include this sentence wherever you want to see the position.
    #     # self.draw(struct_test, stair)
    #
    # def motion1tN(self, structure):
    #     # No collision when pussing actuator 2.
    #     res = structure.push_actuator(0, -20)
    #     self.assertTrue(res)
    #     res = structure.push_actuator(2, -40)
    #     self.assertTrue(res)
    #     inclination = structure.get_inclination()
    #     self.assertAlmostEqual(inclination, 0.0, 4)
    #
    # def motion2tN(self, structure):
    #     # Structure is push to the limit.
    #     res = structure.push_actuator(0, -100)
    #     self.assertTrue(res)
    #     inclination = structure.get_inclination()
    #     self.assertAlmostEqual(inclination, 0.0, 4)

    def testMakeRoomWheel3_no_inclination_limit(self):
        # Check operation fo function make_room_wheel3 on base, when there is
        # not inclination limits (the structure can incline enough).
        landing = 500.0
        stair_list = [
            {'N': 5, 'd': 1000.0, 'w': 250.0, 'h': 80.0}
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
        self.motion1t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion2t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion3t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion4t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion5t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion6t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion7t(struct_test)
        # self.draw(struct_test, stair)
        # NOTE: To see a graphic representation of the structure end position
        # include this sentence wherever you want to see the position.
        # self.draw(struct_test, stair)

    def motion1t(self, structure):
        # No collision when pussing actuator 4.
        res = structure.push_actuator(3, -60)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60.0, 4)

    def motion2t(self, structure):
        # Collision with actuator 3 when pushing actuator 4.
        res = structure.elevate(90)
        self.assertTrue(res)
        res = structure.push_actuator(3, -90)
        self.assertTrue(res)
        res = structure.push_actuator(3, -20)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 29.2857, 4)

    def motion3t(self, structure):
        # Collision with actuator 3 upwards and actuator 1 downwards.
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

    def motion4t(self, structure):
        # Collision with actuator 3 upwards and actuator 1 downwards.
        res = structure.elevate(60)
        self.assertTrue(res)
        res = structure.shift_actuator(1, -60)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -90)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 109.33333, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 52.66666, 4)

    def motion5t(self, structure):
        # Collision with actuator 2 (we need to take wheel 3 onto the first
        # step).
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.advance(140)
        self.assertTrue(res)
        res = structure.push_actuator(2, -80)
        self.assertTrue(res)
        res = structure.advance(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 84.8276, 4)

    def motion6t(self, structure):
        # Collision with actuator 2 upwards and actuator 1 downwards (we need
        # to take wheel 3 onto the first step).
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.advance(140)
        self.assertTrue(res)
        res = structure.push_actuator(2, -80)
        self.assertTrue(res)
        res = structure.advance(130)
        self.assertTrue(res)
        res = structure.push_actuator(0, -70)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 102.5, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 7.5, 4)

    def motion7t(self, structure):
        # Collision just with actuator 3 because it is shift a distance
        # greater than the structure height.
        # NOTE: Although the motion is possible, if we try to shift an actuator
        # a distance greater than the structure height in only one instruction,
        # the funcion make_room_wheel3 can not do it right. See TODO inside
        # this function.
        res = structure.push_actuator(3, -120)
        self.assertFalse(res)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 20.0, 4)

    def testMakeRoomWheel3_with_inclination_limit(self):
        # Check operation fo function make_room_wheel3 on base, when there is
        # inclination limits (the structure inclination is very limited).
        landing = 500.0
        stair_list = [
            {'N': 5, 'd': 1000.0, 'w': 250.0, 'h': 80.0}
        ]
        size = {
            'a': 120.0,
            'b': 150.0,
            'c': 140.0,
            'd': 100.0,
            'h': 2.0,
            'v': 2.0,
            'g': 100.0,
            'n': 60.0}
        wheels = {
            'r1': 25.0,
            'r2': 25.0,
            'r3': 25.0,
            'r4': 25.0}
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion1w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion2w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion3w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion4w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion5w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion6w(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion7w(struct_test)
        # self.draw(struct_test, stair)
        # NOTE: To see a graphic representation of the structure end position
        # include this sentence wherever you want to see the position.
        # self.draw(struct_test, stair)

    def motion1w(self, structure):
        # No collision with any actuator.
        res = structure.push_actuator(3, -70)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60.0, 4)

    def motion2w(self, structure):
        # Collision with actuator 1, but when the collision raises, the
        # structure already has reached its maximum inclination.
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -60)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60.0, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 39.5122, 4)

    def motion3w(self, structure):
        # Collision with actuator 1, but when the collision raises, the
        # structure has not reached yet its maximum inclination. That is, the
        # actuator collision and the maximum inclination error raises at the
        # same time.
        res = structure.elevate(90)
        self.assertTrue(res)
        res = structure.push_actuator(3, -90)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60.0, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 49.5122, 4)

    def motion4w(self, structure):
        # Similar to 3, but the inclination needed by the actuator is less than
        # the maximum, but the inclination needed by the colliding actuator is
        # greater than the maximum.
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -25)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 60.0, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 4.5122, 4)

    def motion5w(self, structure):
        # Similar to 4, but when inclining, the structure collides with
        # actuator 1, so that the maximum inclination is given by the position
        # of actuators 1 and 3.
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(0, -80)
        self.assertTrue(res)
        res = structure.push_actuator(3, -25)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 30.3704, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 14.6296, 4)

    def motion6w(self, structure):
        # Similar to 5, but with actuator 2.
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(1, -80)
        self.assertTrue(res)
        res = structure.push_actuator(3, -25)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 54.6667, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 6.3333, 4)

    def motion7w(self, structure):
        # Similar to 5, but the collision are with actuators 1 and 2.
        res = structure.elevate(100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.advance(140)
        self.assertTrue(res)
        res = structure.push_actuator(2, -80)
        self.assertTrue(res)
        res = structure.advance(100)
        res = structure.push_actuator(0, -90)
        self.assertTrue(res)
        res = structure.push_actuator(3, -90)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 34.1667, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 45.8333, 4)
