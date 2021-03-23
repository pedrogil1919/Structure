"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Definition of the actuator connecting the wheel to the main base.

"""

from enum import Enum

import numpy
import cv2

from physics.wheel_state import MAX_GAP
from physics.wheel import Wheel
from structure.joint import Joint
from control.distance_errors import CollisionErrors


class ActuatorState(Enum):
    ExitUpperBound = 1
    UpperBound = 2
    Center = 3
    LowerBound = 4
    ExitLowerBound = 5
# Possible positions for an actuator:
#   - UpperBound.
#   - LowerBound.
#   - Center: A position between the upper and lower bound.


# =============================================================================
# Wheel actuator definition:
# =============================================================================

class WheelActuator:
    """Class to store actuator information.

    The class also includes its corresponding ending wheel and the joint to the
    structure.

    """

    def __init__(self, position, length, height, radius, base, stairs):
        """Constructor:

        Parameters:
        position -- Horizontal distance with respect to the left side of the
            structure
        length -- Valid range of the actuator.
        height -- Total height (from the top to the center of the wheel).
        radius --Radius of the ending wheel.
        base -- Main structure base that holds the actuator.
        stairs -- Physics structure to check collisions between a wheel and a
          step.
        """
        # Current position and state of the linear actuator.
        self.d = 0.0
        self.state = ActuatorState.LowerBound
        # Valid range of the actuator.
        self.LENGTH = length
        # Total length (from upper joint to the floor).
        self.HEIGHT = height
        # Create a joint to join it to the main base.
        self.JOINT = Joint(base, position)
        # and create the ending wheel.
        # NOTE: The wheel construction checks whether the new created wheel is
        # place in a valid position. If false, it raise a ValueError exception.
        self.WHEEL = Wheel(radius, stairs, self.JOINT.position(height))

    def shift_actuator(self, distance):
        """Shift the actuator and check if the motion is valid.

        Return True when the actuator can reach the required position.
        If not, return False, along with the correction needed to place
        the actuator in a valid position.
        The function can fail when:
          - The actuator reach either the upper or the lower limit.
          - The ending wheel touches the ground.

        However, in case of failure, the function does not place the actuator
        back to its original position, so that the actuator is left in an
        invalid. The calling function have to do the correction.

        Parameters:
        distance -- Distance to move the actuator.

        """
        # Shift actuator.
        # NOTE: This function does not check whether the shift is possible.
        # Calling function MUST check the position, using check_actuator
        # function.
        self.d += distance
        if self.d < -MAX_GAP:
            # The actuator get out of the lower bound (not valid).
            self.state = ActuatorState.ExitLowerBound
        elif self.d > self.LENGTH + MAX_GAP:
            # The actuator reached the upper bound (not valid).
            self.state = ActuatorState.ExitUpperBound
        elif self.d < MAX_GAP:
            self.state = ActuatorState.LowerBound
        elif self.d > self.LENGTH - MAX_GAP:
            self.state = ActuatorState.UpperBound
        else:
            self.state = ActuatorState.Center

    def shift_actuator_proportional(self, height):
        """ Shift the actuator a value proportional to its position.

        This function shift the actuator a value proportional to the given
        height, but also proportional to its position with respect to the whole
        structure.

        This function must be used when inclining the structure. Return True if
        it succeed (see function shift_actuator), or False otherwise. In this
        case, it also returns the proportional value to correct the actuator to
        a valid position.
        Parameters:
        distance -- Absolute distance (the actual value to move is proportional
          to this value.

        """
        # Compute the actual value to move the actuator.
        prop_height = self.JOINT.proportional_lift(height)
        # Move the actuator.
        self.shift_actuator(prop_height)

    def distance_to_stable(self):
        """Compute the distance to place the wheel in a stable position.

        See wheel.distance_to_stable for more info.

        """
        # Compute actual position for the wheel.
        position = self.JOINT.position(self.HEIGHT+self.d)
        # And return the required distance.
        return self.WHEEL.distance_to_stable(position)

    def check_actuator(self):
        """Check if the actuator is in a valid position.

        This function check both, if the actuator is inside is range of
        actuation, and if the wheel is in a valid position.

        Returns:
          - True if everything is in a valid position. False otherwise.
          - If False, returns the horizontal distance to place the wheel back
            to a valid position.
          - If False, returns the vertical distance to place the wheel and the
            actuator back to a valid position. This is the maximum distance
            (in absolute value) between the wheel error and the actuator error.
          - If False, returns also the distance to place the actuator in a
            valid position. If the actuator error is greater than the wheel
            error, this value is the same than the previous value. This is
            needed for incline function, because this function need to
            differenciate between wheel and actuator error.

        """
        # Check if the wheel is in a valid position.
        position = self.JOINT.position(self.HEIGHT+self.d)
        check, h_err, v_err = self.WHEEL.check_wheel(position)
        # Change the sign to the vertical error, since wheel error is measured
        # upwards, while actuator error is downwards (see
        # actuator_sign_criteria.svg):
        v_err = -v_err
        # Check if the actuator has reached one of its bounds.
        a_err = 0.0
        if self.state == ActuatorState.ExitLowerBound:
            # The actuator has gone out of its lower bound.
            check = False
            a_err = -self.d
            v_err = a_err
        elif self.state == ActuatorState.ExitUpperBound:
            # The actuator has gone out if its upper bound. In this case, we
            # have to check also if the wheel is in a valid position with
            # respect to the stair, and get the maximum of both.
            check = False
            a_err = self.LENGTH - self.d
            v_err = min([v_err, a_err])

        return CollisionErrors(check, h_err, a_err, v_err)

    def ground(self):
        """Return True if its ending wheel is lying on the ground."""
        position = self.JOINT.position(self.HEIGHT+self.d)
        return self.WHEEL.ground(position)

    # =========================================================================
    # Control functions.
    # =========================================================================

    def get_wheel_distances(self):
        """Returns the distances of the ending wheel to the stair.

        See stair.set_distances function, and getDistances.svg.

        """
        position = self.JOINT.position(self.HEIGHT+self.d)
        return self.WHEEL.get_distances(position)

    def get_inverse_lift(self, height):
        """Returns the proportional shift for an outer actuator.

        The function computes the shift for an outer actuator when this
        actuator is shfited the given distances when inclining the structure.
        """
        return self.JOINT.inverse_prop_lift(height)
    # =========================================================================
    # Drawing functions.
    # =========================================================================
    # Actuator colors and widths.
    HOUSING_COLOR = (0xB3, 0xB3, 0xB3)
    ACT_COLOR = (0x00, 0x00, 0x00)
    LIMIT_COLOR = (0x00, 0x00, 0xFF)
    ACT_MIDWIDTH = 4
    HOUSING_MIDWIDTH = 10

    def draw(self, origin, image, scale, shift):
        """Draw the actuator."""
        # Get joint position:
        hx0, hy0 = self.JOINT.position(0)
        # Bottom of the housing
        hy1 = hy0-self.LENGTH
        # Top of the actuator.
        hy2 = hy0-self.d
        # Bottom of the actuator.
        hy3 = hy0-self.HEIGHT-self.d

        # Draw actuator wheel.
        self.WHEEL.draw(origin, image, (hx0, hy3), scale, shift)

        # Draw actuator housing:
        cx1 = numpy.float32(scale*(origin[0]+hx0-self.HOUSING_MIDWIDTH))
        cy1 = numpy.float32(scale*(origin[1]-hy0))
        cx2 = numpy.float32(scale*(hx0+origin[0]+self.HOUSING_MIDWIDTH))
        cy2 = numpy.float32(scale*(origin[1]-hy1))
        cv2.rectangle(image, (cx1, cy1), (cx2, cy2), self.HOUSING_COLOR,
                      cv2.FILLED, cv2.LINE_AA, shift)

        # Draw actuator bar:
        cx1 = numpy.float32(scale*(origin[0]+hx0-self.ACT_MIDWIDTH))
        cy1 = numpy.float32(scale*(origin[1]-hy2))
        cx2 = numpy.float32(scale*(hx0+origin[0]+self.ACT_MIDWIDTH))
        cy2 = numpy.float32(scale*(origin[1]-hy3))
        cv2.rectangle(image, (cx1, cy1), (cx2, cy2), self.ACT_COLOR,
                      cv2.FILLED, cv2.LINE_AA, shift)
        # Draw a mark if the actuator is at either end:
        if self.state != ActuatorState.Center:
            px = numpy.float32(scale*(origin[0]+hx0))
            cv2.circle(image, (px, cy1), int(4*scale), self.LIMIT_COLOR, -1,
                       cv2.LINE_AA, shift)

###############################################################################
# End of file.
###############################################################################
