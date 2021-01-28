'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to define the ending wheel of each structure actuator.
'''
import numpy
import cv2

from physics.wheel_state import WheelState

# =============================================================================
# Wheel definition:
# =============================================================================


class Wheel:
    """Class to define the ending wheel of an actuator.
    
    It stores wheel information, and includes functions to interact with the
    physic structure corresponding to the stairs.
    """

    def __init__(self, radius, stairs, position=None):
        """
        Constructor: Save wheel parameters, and check if the wheel is in a
        correct starting position.
        If not in a correct position, raise a ValueError exception, to relocate
        initial position of the structure in a valid one (if possible).
        Parameters:
        radius -- Radius of the wheel.
        structure -- Physical structure for the stairs (for checking purposes).
        position -- Current position of the center (axis) of the wheel. If it
            is in a forbidden position (collision with a step), raise a
            ValueError exception. If position is None, no checking is
            performed.
            
        """
        self.RADIUS = radius
        self.SIMULATOR = stairs
        if position is None:
            self.state = WheelState.Uncheked
            return
        # Check if it is in a valid position, and if so, update its state
        # (normally, it should be ground, although other values can happen).
        self.state, __, __ = stairs.check_collision(position, self.RADIUS)
        if self.state == WheelState.Inside:
            raise ValueError("Wheel in an initial forbidden position.\n \
                Relocate structure.")

    def move_wheel(self, position):
        """Function to call when moving a wheel in any direction.
        
        Check if the wheel can be moved horizontally to a new position,
        that is, the wheel does not collide with any step.
        Return True if the function succeed, and False if not (collision).
        In this case, return the distance the wheel is inside the stair, both
        in horizontal and in vertical direction.
        
        Parameters:
        position -- Coordinates (x,y) for the required center of the wheel.
        
        """
        # Move wheel, and check whether the motion is possible.
        self.state, w, h = \
            self.SIMULATOR.check_collision(position, self.RADIUS)
        if self.state != WheelState.Inside:
            return True, 0, 0
        # If not in a valid position, return False along with the motion
        # required to correct the wheel to a valid position, that is, the
        # distance the wheel is inside the stair.
        return False, w, h

    # TODO: Check if this function is needed (is similar to the one above).
    def lift_wheel(self, position):
        """Move the wheel in vertical direction (see move_wheel).
 
        Returns True if the wheel can be moved, False if not, due to a
        collision with the stair.
        In this case, return the distance the wheel is inside the stair.
        """
        self.state, __, h = \
            self.SIMULATOR.check_collision(position, self.RADIUS)
        if self.state != WheelState.Inside:
            return True, 0
        return False, h

    def ground(self):
        """Check whether the wheel is lying in a horizontal place.
        """
        return self.state == WheelState.Ground or \
            self.state == WheelState.Corner

    def contact(self):
        """Check whether the wheel is in contact with any part of a stair.
        """
        return self.ground() or self.state == WheelState.Contact


    # =========================================================================
    # Drawing functions.
    # =========================================================================
    # Wheel colors:
    AIR_COLOR = (0xFF, 0x00, 0x00)
    GROUND_COLOR = (0x00, 0xFF, 0x00)
    CONTACT_COLOR = (0x00, 0xFF, 0xFF)
    CORNER_COLOR = (0xFF, 0xFF, 0x00)
    UNSTABLE_COLOR = (0x00, 0x00, 0xFF)
    OUTER_COLOR = (0xFF, 0x00, 0xFF)
    OVER_COLOR = (0xFF, 0x80, 0x80)
    INSIDE_COLOR = (0x40, 0x40, 0x40)
    LINE_COLOR = (0x00, 0x00, 0x00)
    LINE_WIDTH = 4

    def draw(self, origin, image, position, scale, shift):
        cx = numpy.float32(scale*(origin[0]+position[0]))
        cy = numpy.float32(scale*(origin[1]-position[1]))
        cr = numpy.int(scale*self.RADIUS)
        if self.state == WheelState.Ground:
            color = self.GROUND_COLOR
        elif self.state == WheelState.Unstable:
            color = self.UNSTABLE_COLOR
        elif self.state == WheelState.Air:
            color = self.AIR_COLOR
        elif self.state == WheelState.Corner:
            color = self.CORNER_COLOR
        elif self.state == WheelState.Contact:
            color = self.CONTACT_COLOR
        elif self.state == WheelState.Outer:
            color = self.OUTER_COLOR
        elif self.state == WheelState.Over:
            color = self.OVER_COLOR
        elif self.state == WheelState.Inside:
            color = self.INSIDE_COLOR
        else:
            raise(ValueError("Wheel state not supported."))
        cv2.circle(image, (cx, cy), cr, color, -1, cv2.LINE_AA, shift)
        cv2.circle(image, (cx, cy), cr, self.LINE_COLOR, 2,
                   cv2.LINE_AA, shift)

###############################################################################
# End of file.
###############################################################################