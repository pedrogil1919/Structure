"""
Created on 29 ene. 2021

@author: pedro.gil@uah.es

The structure can be considered as a set of two sets of a pairs of wheels.
This module define the functionality of the two pairs of wheels.

"""

from math import isinf, inf

from control.distance_errors import merge_collision, StabilityErrors

# When computing the distance for a wheel to move, sometimes we need to give a
# small margin to prevent the wheel to collide with the stair. This is the
# meaning of this margin.
# EDGE_MARGIN = 4.0


class ActuatorPair:

    def __init__(self, rear, front, rear_pair):
        """Constructor:

        Parameters:
        rear -- Rear actuator.
        front -- Front actuator.
        rear_pair -- If true, this is the pair placed in the rear part of the
            structure, the front pair otherwise.

        """
        self.REAR = rear
        self.FRNT = front
        self.REAR_PAIR = rear_pair

    def shift_actuator(self, rear, front, height):
        """Shift the given actuator.

        Parameters:
        rear -- If None, move the rear actuator the distance given. If not
            None but a float, move the actuator this value.
        front -- Same as rear, for the front actuator.
        height: distance to move the actuator when rear and/or front are
          None. Positive value means the the wheel moves away from the
          structure.

        """
        # Rear actuator.
        if rear is not None:
            self.REAR.shift_actuator(rear)
        else:
            self.REAR.shift_actuator(height)
        # Front actuator.
        if front is not None:
            self.FRNT.shift_actuator(front)
        else:
            self.FRNT.shift_actuator(height)

    def shift_actuator_proportional(self, rear, front, height):
        """Shift and actuator a distance proportional to its position.

        This function must be called when inclining the structure. It shift the
        actuator a distance proportional to heihg, but also proportional to its
        position with respect to the structure.

        Parameters:
        rear -- If None, move the rear actuator the distance proportional to
          the height given. If not None but a float, move the actuator this
          value (the actual value, not the proportional one).
        front -- Same as rear, for the front actuator.
        height: distance to proportional move the actuator when rear and/or
          front are None. Positive value means the the wheel moves away from
          the structure.

        """
        # Rear actuator.
        if rear is not None:
            self.REAR.shift_actuator(rear)
        else:
            self.REAR.shift_actuator_proportional(height)
        # Front actuator.
        if front is not None:
            self.FRNT.shift_actuator(front)
        else:
            self.FRNT.shift_actuator_proportional(height)

    def check_collision(self, margin):
        """Check if any of the wheels (or both) are in a forbidden position.

        Parameters:
        margin -- See WheelActuator.check_actuator function.
        
        Returns:
          - False if any of the wheels have collided, True otherwise.
          - If True, returns the distance the pair need to be moved to place
            it in a safe position, both in horizontal and in vertical, and for
            the actuator.

        """
        # Check for possible wheel collisions.
        fr_res = self.FRNT.check_actuator(margin)
        re_res = self.REAR.check_actuator(margin)

        # Merge both data. See merge_collision function for details.
        res = merge_collision(fr_res, re_res)
        # Add the inclination data to complete the information.
        if self.REAR_PAIR:
            # If this is the rear pair, only the front actuator is needed,
            # since the rear actuator is one of the exterior actuator.
            res.add_inclination_errors(
                self.FRNT.get_inverse_lift(fr_res.actuator))
        else:
            # And the opposite.
            res.add_inclination_errors(
                self.REAR.get_inverse_lift(re_res.actuator))
        return res

    def check_stable(self):
        """Check the stability of the pair of wheels.

        This function check if the wheels are in an unstable position (at any
        time, at least one wheel must remain stable).

        Returns:
          - True if the motion succeed. False otherwise.
          - If False, returns the distance the pair need to be moved to place
            it in a safe position. The sign of this distance is always the
            opposite of the distance given.

        """
        # Check if either wheel is in a stable position.
        fr_grd = self.REAR.ground()
        re_grd = self.FRNT.ground()

        if not fr_grd and not re_grd:
            # Both wheels are not on stable:
            # Get the minimum distance the pair has to be moved to place one of
            # the wheels back to a stable position.
            fr_stb = self.FRNT.distance_to_stable()
            re_stb = self.REAR.distance_to_stable()
            # Create a new StabilityErrors object, setting its state to False.
            # Note that if both errors are None, means that None of then are
            # in an unstable position, but rather both has been lifted at the
            # same time, and this is not possible.
            res = StabilityErrors(False, fr_stb, re_stb)
            return res
        return StabilityErrors()

    # =========================================================================
    # Control functions.
    # =========================================================================

    def get_wheel_distances(self):
        """Get distances from wheels to stairs for the control module."""
        # Get distances for both wheels of the pair.
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        # Test if the structure is reaching the end of the stair.
        if isinf(fr_res['wr']):
            # The front wheel has reached the end of the stair.
            if isinf(re_res['wr']):
                # The rear wheel has also reached the end of the stair.
                # NOTE: Check if must be absolute value in the comparison
                # below.
                return 0, inf, min([fr_res['hr'], re_res['hr']])
            else:
                # Only the front wheel has reached the end of the stair. Set
                # the value up to the same than the rear wheel, because then
                # value is not set by the wheel function.
                fr_res['up'] = re_res['up']
                try:
                    fr_res['wc'] = fr_res['wr']
                except KeyError:
                    pass

        # Choose the wheel which is closest to its nearest step. This is the
        # maximum distance the wheel pair can move.
        # Select the active and the passive wheel. Active wheel is the wheel
        # which leads the motion for this pair.
        if re_res['wr'] < fr_res['wr']:
            active = re_res
            passive = fr_res
            index = 0
        else:
            active = fr_res
            passive = re_res
            index = 1
        # Get the horizontal and vertical distances to move.
        hor = active['wr']
        ver = active['hr']

        if re_res['up'] and fr_res['up']:
            # Facing a positive step.
            # Check for possible unstabilities when shifting the wheel.
            if not passive['st']:
                # The passive wheel is not on the ground, so that we can not
                # elevate the active wheel before the passive is on the ground.
                # Divide the instruction into two. The first one will be used
                # to take the passive wheel to the ground, and the second to
                # complete the motion for the active wheel, but this will be
                # done, hopefully, in the following iteration.
                # NOTE: Although the key hc is not always present, for this
                # case, that is, the wheel chosen is the one that is on the
                # ground, it is not possible that the passive wheel is in an
                # unstable position. If this happened, it will be impossible
                # to cross the stair.
                try:
                    # Compute the total height to shift between both actuators.
                    ver_total = abs(active['hr']) + abs(passive['hc'])
                except KeyError:
                    # However, if the key 'hc' does not exist in the passive
                    # wheel, that means that this wheel can not be taken to the
                    # ground. This is likely to happen because both wheel does
                    # not enter in the step, and in this case, the stair can
                    # not be climbed. However, we can try to move the structure
                    # to the edge of the stair and see if it can work.
                    hor = passive['wr']
                    ver = 0.0
                else:
                    # Divide the horizontal distance to advance proportional to
                    # both heights.
                    k = abs(passive['hc']) / ver_total
                    hor = k * active['wr']
                    # Now, take the passive actuator to the ground.
                    ver = passive['hc']
                    # Change the wheel to move.
                    index = (index + 1) % 2
                    # The passive wheel is on the ground, so that we need not
                    # take care of this wheel.
                    # NOTE: Removed (13/05/21). This functionality has been 
                    # moved to wheel.get_wheel_distances function.
                    # if ver > 0:
                        # If the vertical distance is positive, its means that
                        # we are facing a positive step. In this case, add a
                        # small margin to the height to lift the actuator so
                        # that the wheel does not collide with the step.
                    #     ver += EDGE_MARGIN
                    # else:
                        # If we are facing a positive step but the vertical
                        # motion is negative, it means that we are taking the
                        # wheel down once the wheel is already over the step.
                        # In this case, add a small margin to the horizontal
                        # motion so that the wheel lands further than the edge
                        # of the step.
                        # hor += EDGE_MARGIN
        elif not re_res['up'] and not fr_res['up']:
            # Facing a negative step.
            if passive['st']:
                hor = active['wc']
#                 # If the passive wheel is on the ground, check if the active
#                 # wheel can be moved pass the edge of the step.
#                 try:
#                     hor = active['wc']
#                 except KeyError:
#                     pass
            else:
                # Else, if the passive wheel is not on the ground, but the
                # active wheel does, use this instruction to take the passive
                # wheel to the ground as the active wheel moves the distance
                # required (but still stable).
                # Change the active wheel, so that the other wheel can be
                # taking down to the ground.
                index = (index + 1) % 2
                # Note that if the active wheel is stable, the vertical
                # distance must be 0 (you can not take a wheel down if it is
                # on the ground).
                ver = passive['hr']
#                 hor -= EDGE_MARGIN
        else:
            raise NotImplementedError(
                "Wheel facing steps with different sign.")

        return index, hor, ver

    def set_to_ground(self):
        """Return the distance to place both wheels on the ground.

        Returns:
          - Index of the wheel to shift:
            - 0: Rear wheel.
            - 1: Front wheel.
          - Vertical height to shift the actuator.

        However, if both wheel are in the ground, return None as first value.

        """
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        # Rear wheel not on the ground.
        if not re_res['st'] and fr_res['st']:
            return 0, re_res['hr']
        # Front wheel not on the ground.
        elif not fr_res['st'] and re_res['st']:
            return 1, fr_res['hr']
        # Both wheels are on the ground.
        elif fr_res['st'] and re_res['st']:
            return None, 0.0
        else:
            raise RuntimeError("Both wheel on the air.")

    def get_actuator_position(self, index):
        """Returns the shift of an actuator.

        Parameters:
        index -- Index of the actuator:
          - 0: Rear actuator.
          - 1: Front actuator.
        """
        if index == 0:
            return self.REAR.d
        else:
            return self.FRNT.d

    # =========================================================================
    # Drawing functions.
    # =========================================================================

    def position(self, height):
        xr, yr = self.REAR.JOINT.position(height)
        xf, yf = self.FRNT.JOINT.position(height)
        return xr, yr, xf, yf

    def draw(self, origin, image, scale, shift):
        self.FRNT.draw(origin, image, scale, shift)
        self.REAR.draw(origin, image, scale, shift)

    def draw_trajectory(self, origin, image, scale, shift, index):
        if index == 0:
            self.REAR.draw_trajectory(origin, image, scale, shift)
        else:
            self.FRNT.draw_trajectory(origin, image, scale, shift)

###############################################################################
# End of file.
###############################################################################
