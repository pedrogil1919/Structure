'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to define all the physical interaction between the structure and the
exterior elements, such as the stair.
'''

from numpy import float32
import cv2

from physics.wheel_state import WheelState, MAX_GAP

# =============================================================================
# Stair definition:
# =============================================================================


class Stair:
    '''
    Class to define the complete set of steps, and detect collisions with the
    wheels.

    '''
    def __init__(self, stairs, landing=0):
        """
        Constructor: Generate the coordinates of the set of steps.
        See stair_definition_up.svg y stair_definition_down.svg.
        Run draw_stairs.py.

        Parameters:
        stairs -- List of stair parameter. Each element of the list must
            be a dictionary with the following keys:
                - 'd': Posterior landing length (length after the last step).
                - 'w': Horizontal width of the step.
                - 'h': Vertical height of the step.
                - 'N': number of steps for the current stair.
            If more than one stair is defined (len(stairs)>1), the next
            stair starts at the end of the previous one.
        landing -- Initial landing length (make sure it is long enough to place
            the complete wheelchair in a planar surface). Otherwise, an error
            will be raised when building the structure of the wheelchair.

        """
        # Build the stairs:
        # List with the coordinates of the outer corners (upstairs) or
        # inner corners (downstairs).
        self.STAIR = []
        # Horizontal origin.
        x = landing
        y = 0.0

        for s in stairs:
            # Get stair parameters:
            d = s['d']
            c = s['w']
            h = s['h']
            N = int(s['N'])

            for __ in range(N):
                y += h
                self.STAIR.append((x, y))
                x += c
            x += d

        # At the very last step, add an additional element of "landing" size
        # (to allow the structure to end beyond the last step.)
        self.STAIR.append((x, y))

    def find_step(self, p):
        """Find the step over which the wheel is located.

        See figure find_step.svg for more details.

        Returns the coordinates for:
        - height of the step where the wheel lies (yc)
        - horizontal size of this step (xl, xr)
        - height of the previous and next steps (yl, yr)

        Parameters:
        p -- Coordinates (x,y) of the center of the wheel.

        """
        # Horizontal coordinate for the center of the wheel.
        xc = p[0]
        # Consider the height of the first step equal to 0.
        yc = 0.0
        yl = 0.0
        xl = 0.0
        for xs, ys in self.STAIR:
            # Find the step where the center of the wheel lies.
            # NOTE: When we move downstairs, we need to check both greater
            # than or equal to. This happens when we perform a correction
            # when a wheel is placed in an unstable position, so that the
            # wheel move back to a position EQUAL to the edge of the step.
            # In this situation, if none is done, this function will choose
            # incorrect step.
            if ys > yc:
                # Going upstairs:
                if xs > xc:
                    yr = ys
                    xr = xs
                    return yc, xl, xr, yl, yr
            else:
                # Going downstairs:
                if xs >= xc:
                    yr = ys
                    xr = xs
                    return yc, xl, xr, yl, yr

            # Store previous values as the values for the left step.
            xl = xs
            yl = yc
            yc = ys
        # If the code does not reach the return instruction, possibly the
        # wheel is out of the space in which the stairs are defined.
        raise ValueError

    def check_collision(self, p, r):
        """Check a possible collision of a wheel with the stairs.

        Returns:
        - If the wheel is inside the stair (collision), returns Inside
            and the distances to be applied to the wheel center to correct
            such collision.
        - If the wheel is in a correct position, return its state with respect
            to the stair (see WheelState), and None for the other two values.

        Parameters:
        p -- Coordinates of the center of the wheel.
        r -- Wheel radius.

        """

        hc, hl, hr, wl, wr = self.get_distances(p, r)
        # First part. Ensure that there is no collision, but if so, return
        # distances to allow the calling function to correct it.
        # First ensure that the bottom of the wheel is above the step.
        if hc > MAX_GAP:
            # The bottom of the wheel is beneath the ground. Set the wheel on
            # top of step.
            if hr > hl:
                # For upstairs, return also the distance to place the wheel on
                # the previous step.
                return WheelState.Inside, wl-2*r, hc
            else:
                # For downstairs, return also the distance to place the wheel
                # on the next step.
                return WheelState.Inside, -wr+2*r, hc
        # Check if the right edge of the wheel is inside the next step to the
        # right.
        if wr > MAX_GAP and hr > MAX_GAP:
            # The right edge of the wheel is inside the right step. Send both
            # correction options (move to the left or lift). Calling function
            # must choose the appropriate one.
            return WheelState.Inside, -wr, hr

        if wl > MAX_GAP and hl > MAX_GAP:
            # Same as above for the left edge of the wheel.
            return WheelState.Inside, wl, hl

        # At this point, the wheel is in a valid position. Check here the
        # actual state.

        # Check if any point is inside the "contact gap" (ct: contact).
        # Note that we have already check that points are greater that MAX_GAP
        # in the code above (a gap greater than MAX_GAP means the point is
        # inside the stair (forbidden). So here, we only have to check whether
        # the point is greater than -MAX_GAP.
        # Vertical gaps:
        cthc = (hc > -MAX_GAP)               # Bottom of the wheel.
        cthl = (hl > -MAX_GAP) and (wl > 0)  # Bottom-left of the wheel.
        cthr = (hr > -MAX_GAP) and (wr > 0)  # Bottom-right of the wheel.
        # Horizontal gaps:
        ctwl = (MAX_GAP > wl > -MAX_GAP) and (hl > 0)  # Left of the wheel.
        ctwr = (MAX_GAP > wr > -MAX_GAP) and (hr > 0)  # Right of the wheel.

        if cthc:
            # The bottom of the wheel is in contact with the ground:
            if ctwr or ctwl:
                # If the edge of the wheel also touches the step, the wheel is
                # in the corner.
                return WheelState.Corner, 0.0, 0.0
            # Otherwise, the wheel is only in the ground.
            return WheelState.Ground, 0.0, 0.0

        # If the bottom of the wheel is not in contact with the step:
        if ctwr:
            # The wheel is in contact with the vertical part of the next step
            # (to the right)
            return WheelState.Contact, 0.0, 0.0
        if ctwl:
            # The wheel is in contact with the vertical part of the next step
            # (to the left)
            return WheelState.Contact, 0.0, 0.0
        if cthl or cthr:
            # The wheel is in an unstable position.
            return WheelState.Unstable, 0.0, 0.0
        # If none of the above conditions holds, the wheel is on air.
        # Check whether the wheel if just over the outer corner.
        if wr >= -MAX_GAP and hr > hc:
            # The wheel is over an upstairs step.
            return WheelState.Over, 0.0, 0.0
        if wl >= -MAX_GAP and hl > hc:
            # The wheel is over a downstairs step.
            return WheelState.Over, 0.0, 0.0
        # The wheel is not on the corner of any step. Check whether the wheel
        # is higher than the next step. This situation happens when the wheel
        # is lifted to pass one upstairs step.
        # First, check whether we are going:
        #  - a) Upstairs.
        #  - b) Downstairs.
        #  - c) Peak of an upstairs followed by a downstairs.
        #  - d) Valley of a downstairs followed by an upstairs.
        # Among all those case, it is only important when the wheel is facing
        # a positive step, since in this case, we can (if close enough) can
        # advance the structure without taking down the wheel to gain time.
        # In any other case, it is safer to consider the state as Air.
        # In any case, the state is only considered to be higher than the next
        # step if the wheel is close enough to the step. That is, only if the
        # distance is less than the wheel radius.
        if hr > hc and hc >= hl:
            # Case a) Check whether the wheel is above the next step, and the
            # distance to the step is less than its radius.
            if (wr > -r and hr < 0):
                return WheelState.Outer, 0.0, 0.0
        if hr < hc and hc <= hl:
            # Case b) Check whether the wheel is above the previous step.
            return WheelState.Air, 0.0, 0.0
        if hr < hc and hl < hc:
            # Case c) In this case, the wheel can never be considered above
            # the closest step.
            return WheelState.Air, 0.0, 0.0
        if hr > hc and hl > hc:
            # Case d) In this case, the wheel has to be compared against both
            # steps. We compare only with the closest one.
            if wr > wl:
                # In this case, we consider that the structure is facing the
                # next upstairs step.
                if (wr > -r and hr < 0):
                    return WheelState.Outer, 0.0, 0.0
            else:
                return WheelState.Air, 0.0, 0.0
        # In any other case, the wheel is on air.
        return WheelState.Air, 0.0, 0.0

    def get_distances(self, p, r):
        """ Find all the useful distances between the stair and a wheel.

        Returns the following values (see get_distances.svg):
        - hc, hl, hr: Vertical distances from the bottom of the wheel with
            respect to the actual, left and right steps.
        - wl, wr: Horizontal distances from the left (right) edge of the
            wheel to the left (right) step.

        Signs are taken such that a negative value means the wheel is outside
        the stair (valid), and when positive, the wheel is inside the stair
        (forbidden).

        TODO: The wheel is considered a square instead of a circle, so that
        when the wheel is close to an outer corner, the function is not
        working according to a real scenario. Nevertheless, this should not
        be an inconvenience for the proper working of the complete structure
        (see figure detectCollision.svg).

        Parameters:
        p -- Coordinates of the center of the wheel.
        r -- Wheel radius.

        """
        # Find left, central and right step. if the function can not find the
        # step, because the wheel is outside the space where the stairs are
        # defined, this function raises a ValueError exception.
        yc, xl, xr, yl, yr = self.find_step(p)

        # Compute the space between the points of the wheel and the steps.
        # A negative value means this point is outside the stair (correct).
        # A positive value means this point is inside the stair (collision).
        # An value within -MAX_GAP and +MAX_GAP means this point is in contact
        # with the surface of any step.
        # Vertical distances (height)
        hc = yc-p[1]+r  # Bottom of the wheel.
        hl = yl-p[1]+r  # Bottom-left of the wheel (Note, wheel is square).
        hr = yr-p[1]+r  # Bottom-right of the wheel (Note, wheel is square).
        # Horizontal distances (width)
        wl = xl-p[0]+r  # Left of the wheel.
        wr = p[0]+r-xr  # Right of the wheel.

        return hc, hl, hr, wl, wr

    def length(self):
        """Returns the total length of the list of the stairs.

        """
        return self.STAIR[-1][0]

    # =========================================================================
    # Drawing functions.
    # =========================================================================
    
    GROUND_COLOR = (0x00, 0x00, 0x00)
    LINE_SIZE = 2
    CORNER_SIZE = 8
    CORNER_COLOR = (0x00, 0x00, 0xFF)
    
    def draw(self, origin, image, scale, shift):
        """Draw the stair.
        """
        cx1 = float32(scale*origin[0])
        cy1 = float32(scale*origin[1])
        for p in self.STAIR:
            cx2 = float32(scale*(origin[0]+p[0]))
            cy2 = float32(scale*(origin[1]-p[1]))
            cv2.line(image, (cx1, cy1), (cx2, cy1), self.GROUND_COLOR,
                     self.LINE_SIZE, cv2.LINE_AA, shift)
            cv2.line(image, (cx2, cy1), (cx2, cy2), self.GROUND_COLOR,
                     self.LINE_SIZE, cv2.LINE_AA, shift)
            # Draw a point in the step corner.
            cv2.circle(image, (cx1, cy1), self.CORNER_SIZE*shift,
                       self.CORNER_COLOR, -1, cv2.LINE_AA, shift) 
            # Draw steps with different colors.
            # from random import randint
            # color=(randint(0, 0x100), randint(0, 0x100), randint(0, 0x100))
            # cv2.line(image, (cx1, cy1), (cx2, cy1), color, self.LINE_SIZE,
            #    cv2.LINE_AA, self.S)
            # cv2.line(image, (cx2, cy1), (cx2, cy2), color, self.LINE_SIZE,
            #    cv2.LINE_AA, self.S)
            cx1 = cx2
            cy1 = cy2
            
        # Draw the last point of the stair.
        cv2.circle(image, (cx1, cy1), self.CORNER_SIZE*shift,
                   self.CORNER_COLOR, -1, cv2.LINE_AA, shift)
        
###############################################################################
# End of file.
###############################################################################
