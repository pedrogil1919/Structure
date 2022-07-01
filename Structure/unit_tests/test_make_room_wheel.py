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

    def testMakeRoomWheelN_with_inclination_limit(self):
        # Check operation fo function make_room_wheelN on base, when there is
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
            'n': 60.0}
        wheels = {
            'r1': 25.0,
            'r2': 25.0,
            'r3': 25.0,
            'r4': 25.0}
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion1wN(struct_test)
        # self.draw(struct_test, stair)
        # NOTE: To see a graphic representation of the structure end position
        # include this sentence wherever you want to see the position.
        # self.draw(struct_test, stair)

    def motion1wN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 1, so
        # the structure must icline to make room for actuator 0, but the
        # required inclination if greater than the maximum.
        res = structure.push_actuator(0, -90)
        self.assertTrue(res)
        res = structure.push_actuator(0, -30)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -60.0, 4)

    def testMakeRoomWheelN_no_inclination_limit(self):
        # Check operation fo function make_room_wheelN on base, when there is
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
            'n': 800.0}
        wheels = {
            'r1': 25.0,
            'r2': 25.0,
            'r3': 25.0,
            'r4': 25.0}
        stair = stairs.Stair(stair_list, landing)
        structure = base.Base(size, wheels, stair)
        struct_test = deepcopy(structure)
        self.motion1tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion2tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion3tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion4tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion5tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion6tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion7tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion8tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion9tN(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion10tN(struct_test)
        # self.draw(struct_test, stair)
        # NOTE: To see a graphic representation of the structure end position
        # include this sentence wherever you want to see the position.
        # self.draw(struct_test, stair)

    def motion1tN(self, structure):
        # No collision when pussing actuator 2 and 0.
        res = structure.push_actuator(0, -20)
        self.assertTrue(res)
        res = structure.push_actuator(2, -40)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 0.0, 4)

    def motion2tN(self, structure):
        # Structure is pushed to the limit.
        res = structure.push_actuator(0, -100)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, 0.0, 4)

    def motion3tN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 1, so
        # the structure must incline to make room for actuator 0.
        res = structure.push_actuator(0, -90)
        self.assertTrue(res)
        res = structure.push_actuator(0, -30)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -68.3333, 4)

    def motion4tN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 1
        # upwards, and with actuator 3 downwards.
        res = structure.push_actuator(0, -90)
        self.assertTrue(res)
        res = structure.push_actuator(0, -60)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -141.3793, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 8.6207, 4)

    def motion5tN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 1
        # upwards, and with actuator 2 downwards.
        res = structure.push_actuator(0, -90)
        self.assertTrue(res)
        res = structure.push_actuator(2, -70)
        self.assertTrue(res)
        res = structure.push_actuator(0, -60)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -82.0, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 26.0, 4)

    def motion6tN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 2.
        res = structure.advance(130)
        self.assertTrue(res)
        res = structure.push_actuator(3, 80)
        self.assertTrue(res)
        res = structure.advance(150)
        self.assertTrue(res)
        res = structure.push_actuator(2, 80)
        self.assertTrue(res)
        res = structure.push_actuator(0, -40)
        self.assertTrue(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -30.3704, 4)

    def motion7tN(self, structure):
        # Actuator 0 is pushing and the structure collides with actuator 2
        # upwards, and with actuator 3 downwards.
        res = structure.advance(130)
        self.assertTrue(res)
        res = structure.push_actuator(3, 80)
        self.assertTrue(res)
        res = structure.advance(170)
        self.assertTrue(res)
        res = structure.push_actuator(2, 80)
        self.assertTrue(res)
        res = structure.push_actuator(3, -60)
        self.assertTrue(res)
        res = structure.push_actuator(0, -80)
        self.assertTrue(res)
        res = structure.push_actuator(0, -80)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -117.1429, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 62.8571, 4)

    def motion8tN(self, structure):
        # Actuator 1 is pushing the structure upwards, and collides at the same
        # time with actuator 0 and 2. That is, one actuator make the structure
        # incline in one way and the other actuator in the other way. The
        # collision is greater with actuator 2.
        res = structure.advance(130)
        self.assertTrue(res)
        res = structure.push_actuator(3, 80)
        self.assertTrue(res)
        res = structure.advance(170)
        self.assertTrue(res)
        res = structure.push_actuator(2, 80)
        self.assertTrue(res)
        res = structure.incline(-100, fixed=1)
        self.assertTrue(res)
        res = structure.push_actuator(1, -40)
        self.assertTrue(res)
        res = structure.push_actuator(1, -40)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -121.4815, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 15.5556, 4)

    def motion9tN(self, structure):
        # Actuator 1 is pushing the structure upwards, and collides at the same
        # time with actuator 0 and 2. That is, one actuator make the structure
        # incline in one way and the other actuator in the other way. The
        # collision is greater with actuator 0.
        res = structure.advance(130)
        self.assertTrue(res)
        res = structure.push_actuator(3, 80)
        self.assertTrue(res)
        res = structure.advance(170)
        self.assertTrue(res)
        res = structure.push_actuator(2, 80)
        self.assertTrue(res)
        res = structure.push_actuator(1, -40)
        self.assertTrue(res)
        res = structure.incline(-110, fixed=1)
        self.assertTrue(res)
        res = structure.push_actuator(1, -60)
        self.assertFalse(res)
        inclination = structure.get_inclination()
        self.assertAlmostEqual(inclination, -121.4815, 4)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 35.5556, 4)

    def motion10tN(self, structure):
        # Similar to motion8t (testMakeRoomWheel3_no_inclination_limit) but
        # with actuator 0.
        res = structure.advance(150)
        self.assertTrue(res)
        res = structure.push_actuator(3, 80)
        self.assertTrue(res)
        res = structure.advance(170)
        self.assertTrue(res)
        res = structure.push_actuator(2, 80)
        self.assertTrue(res)
        res = structure.push_actuator(0, -100)
        self.assertTrue(res)
        res = structure.push_actuator(0, -50)
        self.assertTrue(res)
        res = structure.push_actuator(0, -20)
        self.assertFalse(res)
    ###########################################################################
    ###########################################################################
    ###########################################################################

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
        struct_test = deepcopy(structure)
        self.motion8t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion9t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion10t(struct_test)
        # self.draw(struct_test, stair)
        struct_test = deepcopy(structure)
        self.motion11t(struct_test)
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
        # Collision just with actuator 3 because it is shifted a distance
        # greater than the structure height.
        # NOTE: Although the motion is possible, if we try to shift an actuator
        # a distance greater than the structure height in only one instruction,
        # the funcion make_room_wheel3 can not do it right. See TODO inside
        # this function.
        res = structure.push_actuator(3, -200)
        self.assertFalse(res)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 100.0, 4)

    def motion8t(self, structure):
        # The difference with the previous test is that, in this case, the
        # structure inclination only collides with actuator 3. This is another
        # problem for the function.
        res = structure.push_actuator(3, -120)
        self.assertFalse(res)
        elevation = res.elevation()
        self.assertAlmostEqual(elevation, 20.0, 4)

    def motion9t(self, structure):
        # In this case, there is a collision with actuator 1 while the initial
        # inclination, but after the second inclination the collision is with
        # the second. In this case, the motion would be possible, but trying
        # to do the motion would cause a collision while simulating step by
        # step, since the structure is close to the edge of both actuators.
        # So, the best solution is just do the same kind of motion, althoug we
        # do not complete the whole actuator shift. The next iteration the
        # motion can be completed. This is not a problem, since this case is
        # too rare to happen.
        res = structure.elevate(80)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.advance(150)
        self.assertTrue(res)
        res = structure.push_actuator(2, -80)
        self.assertTrue(res)
        res = structure.advance(150)
        self.assertTrue(res)
        res = structure.push_actuator(3, -100)
        self.assertTrue(res)
        res = structure.push_actuator(3, -70)
        self.assertTrue(res)
        # In this point, the structure collides with actuators 1 and 2, and so,
        # the motion is not possible if we do the usual motion.
        res = structure.push_actuator(3, -10)
        self.assertFalse(res)

    def motion10t(self, structure):
        # Similar to previous test, but the collision when the initial
        # inclination is with both actuators (1 and 2) with the same distance.
        # In this case, the function colliding_actuator return 1 as the
        # initial actuator, and return 2 for the fixed actuator. This has
        # motivated the NOTE inside base.push_actuator at the end of the
        # function, near the code:
        # "if state4:"
        res = structure.elevate(100.0)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.push_actuator(3, -40)
        self.assertTrue(res)

    def motion11t(self, structure):
        # Similar to previous test, but the collision when the initial
        # inclination is with both actuators (1 and 2) with the same distance.
        # This test have caused the funcion error_distance.colliding_actuator
        # to add the argument check, just to favor to choose actuator 2
        # instead of actuator 1 when trying to check if here we have the same
        # problem as above.
        res = structure.elevate(100.0)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.push_actuator(0, -5.0)
        self.assertTrue(res)
        res = structure.incline(0.1, fixed=1)
        self.assertTrue(res)
        # TODO: This test raises an error because when inclining 0.1, actuator
        # 2 is out of its upper bound (but inside the MAX_GAP limit). However,
        # when pushing actuator 3 here, the structure try go back to set
        # actuator 2 again inside its limits, but in this motion it collides
        # with actuator 3 (by an amount close to MAX_GAP). So, it is an error
        # due to rounding errors that I do not know how to correct it.
        # res = structure.push_actuator(3, -40)
        # self.assertTrue(res)

    ###########################################################################
    ###########################################################################
    ###########################################################################

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

    def test_structure_motion(self):
        """This test is for testing structure motion errors that happens when
        designing the test in this class.

        """
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
        self.structure_motion1(struct_test)

    def structure_motion1(self, structure):
        # When the structure is close to the bounds of the actuators, we can
        # have problems when checking structure position. In this case, the
        # actuator 2 is out of its upper bound, but is inside the actuator
        # margin. So, when trying to incline in the last instruction, since
        # this inclination is not possible, the structure does not return to
        # its initial position, but it tryes to incline back to place the
        # actuator 2 again in a valid (not margin) position. And doing that,
        # the structure collides with the front actuator. However, setting the
        # actuator margin to a value greater than the MAX_GAP value, prevent
        # the function to raise an exception.
        # If we try to set the actuator.MARGIN value to a value equal or less
        # than MAX_GAP, the exceptión is raised (in fact, the value of MARGIN
        #  should be as greater to MAX_GAP as a proportional value obtained
        # from the actuator position, but now is set to its double, which I
        # think is a valid value.
        res = structure.elevate(100.0)
        self.assertTrue(res)
        res = structure.push_actuator(3, -80)
        self.assertTrue(res)
        res = structure.push_actuator(0, -5.0)
        self.assertTrue(res)
        # If we set MARGIN = MAX_GAP an exception is raised, since there is a
        # collision to the front actuator when trying to correct the structure
        # position.
        res = structure.incline(0.1, fixed=1)
        self.assertTrue(res)
