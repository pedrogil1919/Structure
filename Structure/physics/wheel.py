'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to define the ending wheel of each structure actuator. The module
implements the physical interactions with the stairs: collisions and contact.
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

    def move_wheel(self, position):
        """Function to call when moving a wheel in any direction.
        
        Check if the wheel can be moved horizontally to a new position,
        that is, the wheel does not collide with any step.
        
        Return True if the function succeed, and False if not (collision). In
        this case, return the distance the wheel is inside the stair, both
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

#     def get_distances(self, cx, cy):
#         """Computes distances from the wheel to the stair for control module.
#         
#         Arguments:
#         -- cx, cy: Coordinates of the center of the wheel (remember that the
#             coordinates are not stored. They are computed from the actuator
#             position and state).
#         
#         Returns a dictionary with the following keys:
#         -- st: True if the wheel is in the ground, in a stable position.
#         -- hc: Vertical distance from the bottom of the wheel to the ground,
#               when the wheel is in a not unstable position. If not, the key
#               does not exist.
#         -- hr: Vertical distance from the bottom of the wheel to the top of the
#               next step. If the wheel is on an unstable position, it is the
#               distance to the step just beneath the wheel (but not the bottom
#               center of the wheel, since it lies on the previous step).
#         -- wr: Horizontal distance from the wheel to the next step. If the step
#               is positive, is the distance from the right edge of the wheel to
#               the edge of the step. If it is a negative step, is the distance
#               from the left edge of the wheel to the edge of the step. If the
#               wheel is in an unstable position, the key does not exist.
#         -- ws: Horizontal distance that the wheel has to move to place it in
#               a stable position. If it is already stable over a step, is the
#               minimum distance to the next step. If it is on an unstable
#               position, is the distance to the current step. Always with respect
#               to the bottom of the wheel.
#               
#         NOTE: Take into account that horizontal distances are always negative
#         when the wheel is in a valid position. For that reason, a sign change
#         will be needed for almost all the cases.
#         """
# 
#         # Get the distances to the stair (see getDistances.svg).
#         r = self.RADIUS
#         hc, hl, hr, wl, wr = self.SIMULATOR.get_distances((cx, cy), r)
#         # Check if going upstairs, downstairs or the end of the stairs.
#         # TODO: Check if in a change of direction in a double stair.
#         if hr > hc and hc >= hl:
#             # Upstairs direction. The comparison hc = hl happens at the
#             # beginning of the stair.
#             upstairs = True
#         elif hr < hc and hc <= hl:
#             # Downstairs direction.
#             upstairs = False
#         elif hr == hc:
#             # End of the stair.
#             return None
#         else:
#             raise NotImplementedError("Detect when this case happens...")
#             
#         if self.state == WheelState.Ground or \
#                 self.state == WheelState.Corner:
#             # The wheel is on the ground on a stable position.
#             if upstairs:
#                 return {'st': True, 
#                         'hc': hc, 'hr': hr, 'wr': -wr, 'ws': -wr}
#             else:
#                 return {'st': True, 
#                         'hc': hc, 'hr': hr, 'wr': -wr - 2*r, 'ws': -wr}
#         elif self.state == WheelState.Outer or \
#                 self.state == WheelState.Over or \
#                 self.state == WheelState.Unstable:
#             if upstairs:
#                 return {'st': False, 
#                         'hr': hr, 'wr': -wr + r, 'ws': -wr + r}
#             else:
#                 raise NotImplementedError()
# #                 # This case will never happen.
# #                 # TODO: Review. I just only copy - paste from above.
# #                 return {'st': False, 
# #                         'hr': hr, 'wr': -wl + r, 'ws': -wl + r}
#                 
#         elif self.state == WheelState.Air or \
#                 self.state == WheelState.Contact:
#             if upstairs:
#                 return {'st': False, 
#                         'hc': hc, 'hr': hr, 'wr': -wr, 'ws': -wr}
#             else:
#                 raise NotImplementedError()
# #                 # This case will never happen.
# #                 # TODO: Review. I just only copy - paste from above.
# #                 return {'st': False, 
# #                         'hc': hc, 'hr': hr, 'wr': -wl, 'ws': -wl}

    def back_to_stable(self, cx, cy):
        """Return the distance needed to place the wheel in a stable position.
        
        Compute the distance between the bottom of the wheel and the outer
        corner of a step. If the structure is moved this distance, the wheel
        will be stable. This function works only when the wheel is in an
        unstable position.
        
        """
        #TODO
        if self.state != WheelState.Unstable:
            return 0
        # Compute the distance for a radius equal 0. With this trick, the
        # function returns the desired 
        hc, hl, hr, wl, wr = self.SIMULATOR.get_distances((cx, cy), 0)
        if hr > hc and hc >= hl:
            # Upstairs direction. The comparison hc = hl happens at the
            # beginning of the stair.
            return wr
        elif hr < hc and hc <= hl:
            # Downstairs direction.
            return wl
        return 0
           
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