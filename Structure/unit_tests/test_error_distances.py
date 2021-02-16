'''
Created on 5 feb. 2021

@author: pedro
'''
import unittest
import numpy
import cv2

from physics import stairs
from structure import base

class ErrorDistancesTest(unittest.TestCase):
    
    def testCollisions(self):
        """Test if the structure returns the correct distance, included its
        sign when a wheel collides with a step.
        
        """
        # Positive steps.
        landing = 200.0
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

        # Check simple wheel collision.
        res = base_test.shift_actuator(2, 5.0)
        self.assertFalse(res, "Error in base.shift. Both wheel on air.")
        self.assertEqual(res.central, -5.0, "Error in base.shift. Error distance wrong")

        # Check simple actuator collision.
        res = base_test.shift_actuator(2, -5.0)
        self.assertFalse(res, "Error in base.shift. Both wheel on air.")
        self.assertEqual(res.actuator, 5.0, "Error in base.shift. Error distance wrong")

        res = base_test.elevate(10.0)
        self.assertTrue(res, "Error in base.elevate. Wrong collision detected.")
        # Check that both wheel of a pair can be in the air at the same time.
        res = base_test.shift_actuator(3, -5.0)
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.shift_actuator(2, -5.0)
        self.assertFalse(res, "Error in base.shift. Both wheel on air.")
        self.assertEqual(res.actuator, 5.0, "Error in base.shift. Error distance wrong")
        # Take the wheel to the ground.
        res = base_test.shift_actuator(3, 10.0)
        self.assertFalse(res, "Error in base.shift. No collision detected.")
        self.assertEqual(res.central, -5.0, "Error in base.shift. Error distance wrong")
        # Correct the distance required by the error returned by the function.
        # This should place the wheel on the ground.
        res = base_test.shift_actuator(3, 10.0 + res.central)
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        
        # Simple collision.
        res = base_test.advance(36.0)
        self.assertFalse(res, "Error in base.advance. No collision detected.")
        self.assertEqual(res.horizontal, -1.0, 
                         "Error in base.advance. Error distance wrong")
        res = base_test.advance(36.0 + res.horizontal)
        self.assertTrue(res, "Error in base.advance. Wrong collision detected.")
        # Double collision. Both wheel get inside the first step.
        # NOTE: This kind of double collision will not be a normal case, for
        # generating this collision the structure must advance a large distance,
        # but normally the structure will advance smaller distances. An error
        # will be raised when advancing a distance greater than the step, since
        # the wheels will be located 2 steps further than there was, so that
        # the error returned will be refered to the second step, and not the
        # first, so that the distance returned will not place the wheel in
        # the appropiate location. However, for this error to happen, we have
        # to advance the structure a distance larger than the step, and this
        # should not happen ever.
        res = base_test.advance(35.0)
        self.assertFalse(res, "Error in base.advance. No collision detected.")
        self.assertEqual(res.horizontal, -35.0, 
                         "Error in base.advance. Error distance wrong")
         
        res = base_test.elevate(40.0)
        self.assertTrue(res, "Error in base.elevate. Wrong collision detected.")
        res = base_test.shift_actuator(3, -40.0)
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.advance(15.0)
        # Prepare a wheel unstability when inclining.
        res = base_test.shift_actuator(2, -15)
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
         
        # Now, when inclining the structure, the first wheel will move back,
        # so that it gets to an unstable position, and since the second wheel
        # is on air, the motion is not posible.
        res = base_test.shift_actuator(1, -10)
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.incline(10.0)
        self.assertFalse(res, "Error in base.incline. No collision detected.")
        # However, if we move the structure the horizontal distance returned
        # by the function in opposite direction, now the wheel will not gets
        # to an unstable position.
        res = base_test.advance(-res.horizontal)
        res = base_test.incline(10.0)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        # Now the front wheel is at the limit, so that moving the structure
        # back is not possible.
        res = base_test.advance(-5.0)
        self.assertFalse(res, "Error in base.incline. Wrong collision detected.")
        self.assertEqual(res.horizontal, -5.0, 
                         "Error in base.advance. Error distance wrong")
        
        res = base_test.shift_actuator(2, -30.0)                
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.advance(40.0)
        self.assertTrue(res, "Error in base.advance. Wrong collision detected.")
        res = base_test.shift_actuator(2, 20.0)                
        self.assertFalse(res, "Error in base.shift. No collision detected.")
        res = base_test.shift_actuator(2, 20.0+res.central)                
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.shift_actuator(1, -30.0)                
        self.assertTrue(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.incline(25.0)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.advance(50.0)
        self.assertFalse(res, "Error in base.advance. Wrong collision detected.")
        res = base_test.advance(50.0+res.horizontal)
        res = base_test.shift_actuator(3, -50.0)                
        self.assertFalse(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.shift_actuator(3, -50.0+res.actuator) 
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.advance(50.0)
        self.assertFalse(res, "Error in base.advance. Wrong collision detected.")
        res = base_test.advance(50.0+res.horizontal)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        
        res = base_test.shift_actuator(3, 30.0)                
        self.assertFalse(res, "Error in base.shift. Wrong collision detected.")
        res = base_test.shift_actuator(3, 30.0+res.central) 
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.shift_actuator(2, -40.0)                
        self.assertFalse(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.shift_actuator(2, -40.0+res.central) 
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.incline(10.0)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.shift_actuator(2, -40.0)                
        self.assertFalse(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.shift_actuator(2, -40.0+res.central) 
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.advance(50.0)
        self.assertFalse(res, "Error in base.advance. Wrong collision detected.")
        res = base_test.advance(50.0+res.horizontal)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        # Collide the rear wheel when inclining the structure. In this case,
        # we fix the front wheel, wo that the rear wheel advance forward and
        # collides with the step. In this case, the error must be negative.
        res = base_test.incline(10.0, False, True)
        self.assertFalse(res, "Error in base.advance. Wrong collision detected.")
        self.assertTrue(res.horizontal, "Error in base.incline. Wrong collision detected.")
        res = base_test.advance(res.horizontal)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")
        res = base_test.incline(10.0, False, True)
        self.assertTrue(res, "Error in base.incline. Wrong collision detected.")

        image = numpy.full((500, 800, 3), 0xFF, numpy.uint8)
        stair_test.draw((0, 400), image, 8, 3)
        base_test.draw((0, 400), image, 8, 3)
        cv2.imshow("image", image)
        cv2.waitKey()        
        
    def testActuatorError(self):
        """Check if the actuator returns the correct value when both the
        actuator and the wheel have errors.
        
        """
        landing = 200.0
        stair_list = [
            {'N': 2, 'd': 250.0, 'w': 100.0, 'h': -40.0}
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
        
        res, __ = base_test.advance(75)
        self.assertTrue(res, "Error in advance")
        res, __ = base_test.elevate(20)
        self.assertTrue(res, "Error in elevate")
        res, __ = base_test.shift_actuator(3, 25)
        self.assertTrue(res, "Error in shift actuator")
        # The front wheel is 15 above the step. The actuator 5 to its lower
        # boound. 
        res, err = base_test.shift_actuator(3, 10)
        self.assertFalse(res, "Error in shift actuator")
        self.assertEqual(err, -5.0, "Error in shift actuator")
        
        # Example when the actuator has a greater error than the wheel.
        res, err = base_test.shift_actuator(3, 20)
        self.assertFalse(res, "Error in shift actuator")
        self.assertEqual(err, -15.0, "Error in shift actuator")
        # NOTE: If we correct the required distance by the amount returned by
        # the function, the actuator must be placed in its lower limit.
        res, err = base_test.shift_actuator(3, 20 + err)

        # Now, prepare a case when the wheel is closer to the ground than the
        # actuator to its lower limit.
        res, __ = base_test.elevate(-15)
        self.assertTrue(res, "Error in elevate")
        res, err = base_test.shift_actuator(3, 20)
        self.assertFalse(res, "Error in shift actuator")
        self.assertEqual(err, -10.0, "Error in shift actuator")
        # Again, if we correct the distance by the amount returned, the wheel
        # must be placed on the ground.
        res, err = base_test.shift_actuator(3, 20 + err)

        
#         image = numpy.full((500, 800, 3), 0xFF, numpy.uint8)
#         stair_test.draw((0, 400), image, 8, 3)
#         base_test.draw((0, 400), image, 8, 3)
#         cv2.imshow("image", image)
#         cv2.waitKey()        


    def testName(self):
        pass

