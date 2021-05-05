'''
Created on 26 ene. 2021
@author: pedro.gil@uah.es
Test stairs constructor.
'''

import unittest

from physics import stairs

class StairTest(unittest.TestCase):

    def testUpstairs(self):
        """Check if the stair generated has the correct size.
        """
        landing = 100.0
        stair_list = [
            {'N': 2, 'd': 250.0, 'w': 50.0, 'h': +25.0}
            ]
        # For this definition, check the coordinates of the last point of the
        # stair.
        stairs_test = stairs.Stair(stair_list, landing)

        self.assertEqual(stairs_test.STAIR[-1][0], 450.0,
                         "Error: Stair construction failed.")
        self.assertEqual(stairs_test.STAIR[-1][1], 50.0,
                         "Error: Stair construction failed.")

    def testDownstairs(self):
        """Check if the stair generated has the correct size.
        """
        # Same as testUpstairs for negative steps.
        landing = 100.0
        stair_list = [
            {'N': 3, 'd': 250.0, 'w': 50.0, 'h': -25.0}
            ]
        stairs_test = stairs.Stair(stair_list, landing)

        self.assertEqual(stairs_test.STAIR[-1][0], 500.0,
                         "Error: Stair construction failed.")
        self.assertEqual(stairs_test.STAIR[-1][1], -75.0,
                         "Error: Stair construction failed.")

    def testFindStepUp(self):
        """Check localization of the step where a wheel lies (upstairs)
        """
        landing = 100.0
        stair_list = [
            {'N': 3, 'd': 250.0, 'w': 50.0, 'h': +25.0}
            ]
        stairs_test = stairs.Stair(stair_list, landing)
        # Middle of the first step.
        yc, xl, xr, yl, yr = stairs_test.find_step((125, 0))
        self.assertEqual(yc,  25.0, "Error: Find Step failed.")
        self.assertEqual(xl, 100.0, "Error: Find Step failed.")
        self.assertEqual(xr, 150.0, "Error: Find Step failed.")
        self.assertEqual(yl,   0.0, "Error: Find Step failed.")
        self.assertEqual(yr,  50.0, "Error: Find Step failed.")
        # Just edge of the second step.
        yc, xl, xr, yl, yr = stairs_test.find_step((150, 0))
        self.assertEqual(yc,  50.0, "Error: Find Step failed.")
        self.assertEqual(xl, 150.0, "Error: Find Step failed.")
        self.assertEqual(xr, 200.0, "Error: Find Step failed.")
        self.assertEqual(yl,  25.0, "Error: Find Step failed.")
        self.assertEqual(yr,  75.0, "Error: Find Step failed.")
        # Over the last step.
        yc, xl, xr, yl, yr = stairs_test.find_step((215, 0))
        self.assertEqual(yc,  75.0, "Error: Find Step failed.")
        self.assertEqual(xl, 200.0, "Error: Find Step failed.")
        self.assertEqual(xr, 500.0, "Error: Find Step failed.")
        self.assertEqual(yl,  50.0, "Error: Find Step failed.")
        self.assertEqual(yr,  75.0, "Error: Find Step failed.")
        # Out of the stair definition
        self.assertRaises(ValueError, stairs_test.find_step, (600, 0))

    def testFindStepDown(self):
        """Check localization of the step where a wheel lies (upstairs)
        """
        landing = 100.0
        stair_list = [
            {'N': 3, 'd': 250.0, 'w': 50.0, 'h': -25.0}
            ]
        stairs_test = stairs.Stair(stair_list, landing)
        # Middle of the first step.
        yc, xl, xr, yl, yr = stairs_test.find_step((125, 0))
        self.assertEqual(yc, -25.0, "Error: Find Step failed.")
        self.assertEqual(xl, 100.0, "Error: Find Step failed.")
        self.assertEqual(xr, 150.0, "Error: Find Step failed.")
        self.assertEqual(yl,   0.0, "Error: Find Step failed.")
        self.assertEqual(yr, -50.0, "Error: Find Step failed.")
        # Just edge of the second step (note that the behavior changes in
        # this case).
        yc, xl, xr, yl, yr = stairs_test.find_step((150, 0))
        self.assertEqual(yc, -25.0, "Error: Find Step failed.")
        self.assertEqual(xl, 100.0, "Error: Find Step failed.")
        self.assertEqual(xr, 150.0, "Error: Find Step failed.")
        self.assertEqual(yl,   0.0, "Error: Find Step failed.")
        self.assertEqual(yr, -50.0, "Error: Find Step failed.")
        # Over the last step.
        yc, xl, xr, yl, yr = stairs_test.find_step((215, 0))
        self.assertEqual(yc, -75.0, "Error: Find Step failed.")
        self.assertEqual(xl, 200.0, "Error: Find Step failed.")
        self.assertEqual(xr, 500.0, "Error: Find Step failed.")
        self.assertEqual(yl, -50.0, "Error: Find Step failed.")
        self.assertEqual(yr, -75.0, "Error: Find Step failed.")
        # Out of the stair definition
        self.assertRaises(ValueError, stairs_test.find_step, (600, 0))
                         
###############################################################################
# End of file.
###############################################################################