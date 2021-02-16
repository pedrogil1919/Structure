'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to define the ending wheel of each structure actuator. The module
implements the physical interactions with the stairs: collisions and contacts.

'''

import numpy
import cv2
from math import inf

from physics.wheel_state import WheelState, MAX_GAP

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
        correct starting position (no collision, but it can be on air).
        
        If not in a correct position, raise a ValueError exception to warn the
        user to define correctly the starting position.
        
        The absolute position of the wheel is not stored here, it is computed
        from the structure base and actuator positions.
        
        Parameters:
        radius -- Radius of the wheel.
        stairs -- Physical structure for the stairs (for checking purposes).
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

    def check_wheel(self, position):
        """Function to check the position of a wheel with respect to the stair.
        
        Check if the wheel can be placed in the required position, and no
        collision happens. The function also update the whell state (see
        WheelState enum).
        
        Return True if the function succeed, and False if not (collision). In
        this case, returns the distance the wheel is inside the stair, both
        in horizontal and in vertical direction (run test_wheel_state.py).
        
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

    def distance_to_stable(self, position):
        """Return the distance needed to place the wheel in a stable position.
        
        Compute the distance between the bottom of the wheel and the outer
        corner of a step. If the structure is moved this distance, the wheel
        will be stable. This function works only when the wheel is in
        unstable state.
        
        Parameters:
        position -- Coordinates (x,y) for the required center of the wheel.
        
        """
        #TODO
        if self.state != WheelState.Unstable:
            # If the wheel is not in an unstable position, this value has no
            # sense (is not useful at all).
            return None
        
        # Compute the distance for a radius equal 0. With this trick, the
        # function returns the desired distance.
        hc, hl, hr, wl, wr = self.SIMULATOR.get_distances(position, 0)
        if hr > hc and hc >= hl:
            # Upstairs direction. The comparison hc = hl happens at the
            # beginning of the stair.
            return wr
        elif hr < hc and hc <= hl:
            # Downstairs direction.
            return wl
        return None

    def ground(self, position):
        """Check whether the wheel is lying in a horizontal place.
         
        """
        self.check_wheel(position)
        return self.state == WheelState.Ground or \
            self.state == WheelState.Corner

    # =========================================================================
    # Control functions.
    # =========================================================================
    
    def get_distances(self, position):
        """Computes distances from the wheel to the next step.
         
        Arguments:
        -- position: Coordinates of the center of the wheel (remember that the
            coordinates are not stored. They are computed from the actuator
            position and state).
         
        Returns a dictionary with the following keys (see get_distances folder):
        -- up: True if the next step is positive (upstairs).
        -- st: True if the wheel is in the ground, in a stable position.
        -- hr: Vertical distance from the bottom of the wheel to the top of the
            next step. If the wheel is on an unstable position, it is the
            distance to the step just beneath the wheel (not to the step just
            beneath the center of the wheel, since this step is the previous
            one).
        -- hc: Vertical distance from the bottom of the wheel to the ground,
            when the wheel is in a not unstable position nor in the ground.
            If not, the key does not exist.
        -- wr: Horizontal distance from the wheel to the next step. If the step
            is positive and the wheel is below the next step, is the distance
            from the right edge of the wheel to the edge of the step. If the
            wheel is above the next step, is the distance to place the wheel
            on a stable position in the next step.
            If it is a negative step and the wheel is not on the ground, is
            the distance to place the center of the wheel just in the outer
            corner of the step. If it is on the ground, is the distance to
            place the left edge of the wheel further from the current step
            (so that if we take the wheel down, the wheel will be placed on
            the ground in a stable position in the next step). To ensure that
            the wheel get the state Air instead of Unstable, a small value
            (MAX_GAP) is added to the distance to be returned.
        -- wc: Only for negative steps and when the wheel is on the ground,
            this is the value to place the wheel just in the outer corner of
            the step. This is an alternative distance for when the other wheel
            of the pair is not in the ground, so that moving this wheel to
            this position ensures that never both wheel will be on air.
               
        NOTE: Take into account that horizontal distances are always negative
        when the wheel is in a valid position. For that reason, a sign change
        will be needed for almost all the cases.
        """
 
        # Get the distances to the stair (see getDistances.svg).
        r = self.RADIUS
        hc, hl, hr, wl, wr = self.SIMULATOR.get_distances(position, r)
        
        res = {'st': self.ground(position)}
        # Check if going upstairs, downstairs or the end of the stairs.
        # TODO: Check if in a change of direction in a double stair.
        ########################################################################
        # Upstairs:
        ########################################################################
        if hr > hc and hc >= hl:
            res['up'] = True
            # Upstairs direction. The comparison hc = hl happens at the
            # beginning of the stair.
            if self.state == WheelState.Air:
                res['hc'] = hc
                res['hr'] = hr + MAX_GAP
                res['wr'] = -wr
            elif self.state == WheelState.Contact:
                res['hc'] = hc
                res['hr'] = hr + MAX_GAP
                res['wr'] = 0.0
            elif self.state == WheelState.Corner:
                res['hr'] = hr + MAX_GAP
                res['wr'] = 0.0
            elif self.state == WheelState.Ground:
                res['hr'] = hr + MAX_GAP
                res['wr'] = -wr
            elif self.state == WheelState.Outer:
                res['hr'] = hr
                res['hc'] = hc
                res['wr'] = -wr + r + MAX_GAP
            elif self.state == WheelState.Over:
                res['hr'] = hr
                res['wr'] = -wr + r + MAX_GAP
            elif self.state == WheelState.Unstable:
                res['hr'] = 0.0
                res['wr'] = -wr + r + MAX_GAP            
        ########################################################################
        # Downstairs:
        ########################################################################
        elif hr < hc and hc <= hl:
            # Downstairs direction.
            res['up'] = False
            if self.state == WheelState.Air:
                res['hr'] = hc
                res['wr'] = -wr - MAX_GAP
                res['wc'] = -wr + 2*r + MAX_GAP
            elif self.state == WheelState.Contact:
                res['hr'] = hc
                res['wr'] = -wr - MAX_GAP
                res['wc'] = -wr + 2*r + MAX_GAP
            elif self.state == WheelState.Corner:
                res['hr'] = 0.0
                res['wr'] = -wr + r
                res['wc'] = -wr + 2*r + MAX_GAP
            elif self.state == WheelState.Ground:
                res['hr'] = 0.0
                res['wr'] = -wr + r
                res['wc'] = -wr + 2*r + MAX_GAP
            elif self.state == WheelState.Outer:
                raise NotImplementedError("It should not happen")
            elif self.state == WheelState.Over:
                res['hr'] = hc
                res['wr'] = -wr - MAX_GAP
                res['wc'] = -wr + 2*r + MAX_GAP
            elif self.state == WheelState.Unstable:
                res['hr'] = 0.0
                res['wr'] = -wr + r
                res['wc'] = -wr + 2*r + MAX_GAP 
        ########################################################################
        # End:
        ########################################################################
        elif hr == hc:
            # The wheel has reached the end of the stair. Send an infinite value
            # to warn the calling function.
            res['wr'] = inf
            if self.state == WheelState.Air:
                res['hr'] = hc
            elif self.state == WheelState.Contact:
                res['hr'] = hc
            elif self.state == WheelState.Corner:
                res['hr'] = hc
            elif self.state == WheelState.Ground:
                res['hr'] = hc
            elif self.state == WheelState.Outer:
                res['hr'] = hc
            elif self.state == WheelState.Over:
                res['hr'] = hl
                res['wc'] = wl - MAX_GAP
            elif self.state == WheelState.Unstable:
                res['hr'] = 0.0
                res['wc'] = wl + MAX_GAP

        else:
            raise NotImplementedError("Detect when this case happens...")
        
        return res

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

