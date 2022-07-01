"""
Created on 29 ene. 2021

@author: pedro.gil@uah.es

The structure can be considered as a set of two sets of a pairs of wheels.
This module define the functionality of the two pairs of wheels.
"""

from simulator.error_distance import InclineActuatorError
from simulator.error_distance import PairError


class ActuatorPair:
    """Define the behaviour of a pair of actuators."""

    def __init__(self, rear, front):
        """Constructor:

        Arguments:
        rear -- Rear actuator.
        front -- Front actuator.

        """
        self.REAR = rear
        self.FRNT = front

    def shift_actuator(self, rear, front, height):
        """Shift the given actuator.

        Argumetns:
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

        This function must be called when inclining the structure. It shifts
        the actuator a distance proportional to height, but also proportional
        to its position with respect to the structure.

        Arguments:
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

    def check_collision(self):
        """Check if any of the wheels (or both) are in a forbidden position.

        Return:
          - ActuatorError object for the external actuator.
          - InternalActuatorError object for the internal actuator.
          - PairError object.
        """
        # Check for possible wheel collisions.
        re_col = self.REAR.check_actuator()
        fr_col = self.FRNT.check_actuator()

        # Check if the pair of wheels are in a stable position.
        # NOTE: This checking must be done here, to have the info available
        # for the next part of the function, that is, check for inclination
        # errors.
        re_pair = self.check_stable()

        if not re_col:
            re_inc_error = self.REAR.get_inverse_prop_lift(re_col.vertical)
            # And add the inclination height.
            re_adv_error = self.REAR.get_inverse_prop_lift(
                self.REAR.get_lift_from_horizontal_motion(
                    re_col.horizontal))
            # Remember that the funcion get_inverse_prop_lift return a value
            # for each actuator, but in this case, we only need the value for
            # the actuator 0.
            re_adv_inc = re_adv_error[0]
            # Create a new object from the appropriate class.
            re_col = InclineActuatorError(re_col, re_inc_error, re_adv_inc)
        if not fr_col:
            fr_inc_error = self.FRNT.get_inverse_prop_lift(fr_col.vertical)
            # And add the inclination height.
            fr_adv_error = self.FRNT.get_inverse_prop_lift(
                self.FRNT.get_lift_from_horizontal_motion(
                    fr_col.horizontal))
            fr_adv_inc = fr_adv_error[0]
            # Create a new object from the appropriate class.
            fr_col = InclineActuatorError(fr_col, fr_inc_error, fr_adv_inc)

        return re_col, fr_col, re_pair

        # # Add the inclination data to complete the information.
        # if self.REAR_PAIR:
        #     if not fr_col:
        #         # Add rear and front heights.
        #         incline_error = self.FRNT.get_inverse_lift(fr_col.vertical)
        #         # And add the inclination height.
        #         __, incline = self.FRNT.get_inverse_lift(
        #             self.FRNT.get_lift_from_horizontal_motion(
        #                 fr_col.horizontal))
        #         # Create a new object from the appropriate class.
        #         fr_col = InclineActuatorError(fr_col, incline_error, incline)
        # else:
        #     # For the front pair, we have to add:
        #     if not re_col:
        #         # the rear and front heights, plus the inclination height to
        #         # the rear actuator, since this is the internal one.
        #         rear, front = self.REAR.get_inverse_lift(re_col.vertical)
        #         __, incline = self.REAR.get_inverse_lift(
        #             self.REAR.get_lift_from_horizontal_motion(
        #                 re_col.horizontal))
        #         re_col = InclineActuatorError(re_col, rear, front, incline)
        #     if not fr_col:
        #         # And for the front actuator, only have to include the
        #         # inclination height.
        #         incline = self.FRNT.get_lift_from_horizontal_motion(
        #             fr_col.horizontal)
        #         fr_col = InclineActuatorError(fr_col, incline)
        #
        # return re_col, fr_col, re_pair

    def check_stable(self):
        """Check the stability of the pair of wheels.

        This function check if the wheels are in an unstable position (at any
        time, at least one wheel must remain stable).

        Return:
          - True if the motion succeed. False otherwise.
          - If False, returns the distance the pair need to be moved to place
            it in a safe position. The sign of this distance is always the
            opposite of the distance given.
        """
        # Check if either wheel is in a stable position.
        # Check possible pair unstability:
        re_stb = self.REAR.check_stable()
        fr_stb = self.FRNT.check_stable()
        # Take into account that a pair is unstable when both wheels are not
        # in a stable position in the ground.
        if not re_stb and not fr_stb:
            # In this case, the pair is unstable.
            if re_stb.horizontal is not None:
                # Compute the inclination needed to place the wheel back to a
                # stable position.
                re_incline = self.REAR.get_lift_from_horizontal_motion(
                    re_stb.horizontal)
                # And compute the proportional inclination for all the
                # actuators.
                re_inc_error = self.REAR.get_inverse_prop_lift(re_incline)
                # Take into account that only the inclination from the rear
                # actuator (or from the front actuator changing sign) is
                # needed. So, we choose the firt value of the array.
                re_inc_inc = re_inc_error[0]
            else:
                re_inc_inc = None
            if fr_stb.horizontal is not None:
                # Do the same for the front actuator.
                fr_incline = self.FRNT.get_lift_from_horizontal_motion(
                    fr_stb.horizontal)
                fr_inc_error = self.FRNT.get_inverse_prop_lift(fr_incline)
                fr_inc_inc = fr_inc_error[0]
            else:
                fr_inc_inc = None
            return PairError(re_stb, fr_stb, re_inc_inc, fr_inc_inc)
        else:
            return PairError(re_stb, fr_stb)

        # TODO: He sustituido este código por que el hay arriba, ya que antes
        # tenía que diferenciar si se trataba del par de atrás o el par de
        # delante, pero ahora ya no es necesario.
        """
            if self.REAR_PAIR:
                # If the wheel is horizontally unstable (that is, is over the
                # corner of the step), compute the inclination we have to do
                # to move the wheel back to a stable position (this is not
                # always possible).
                if fr_stb.horizontal is not None:
                    fr_inc_error = self.FRNT.get_inverse_prop_lift(
                        self.FRNT.get_lift_from_horizontal_motion(
                            fr_stb.horizontal))
                    fr_inc_inc = fr_inc_error[0]
                else:
                    fr_inc_inc = None
                re_inc_inc = None
            else:
                if re_stb.horizontal is not None:
                    re_inc_error = self.REAR.get_inverse_prop_lift(
                        self.REAR.get_lift_from_horizontal_motion(
                            re_stb.horizontal))
                    re_inc_inc = re_inc_error[0]
                else:
                    re_inc_inc = None
                if fr_stb.horizontal is not None:
                    fr_inc_error = self.FRNT.get_lift_from_horizontal_motion(
                        fr_stb.horizontal)
                    fr_inc_inc = fr_inc_error[0]
                else:
                    fr_inc_inc = None
            return PairError(re_stb, fr_stb, re_inc_inc, fr_inc_inc)
        else:
            return PairError(re_stb, fr_stb)
        """
    # =========================================================================
    # Control functions.
    # =========================================================================

    def get_wheel_distances(self):
        """Get distances from wheels to stairs for the control module."""
        # Get distances for both wheels of the pair.
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        # Test if the structure is reaching the end of the stair.
        if fr_res['end']:
            # The front wheel has reached the end of the stair.
            if re_res['end']:
                # The rear wheel has also reached the end of the stair.
                # NOTE: Check if must be absolute value in the comparison
                # below.
                return 0, fr_res['wr'], min([fr_res['hr'], re_res['hr']]), True
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
                try:
                    passive_ver = passive['hc']
                except KeyError:
                    # However, if the key 'hc' does not exist in the passive
                    # wheel, that means that this wheel can not be taken to the
                    # ground. This is likely to happen because both wheel does
                    # not enter in the step, and in this case, the stair can
                    # not be climbed. However, we can try to move the structure
                    # to the edge of the step and see if it works. If it does
                    # not work, the stair can not be climbed because both
                    # wheels do not enter in the step at the same time.
                    hor = passive['wr']
                    ver = passive['hr']
                else:
                    # Compute the total height to shift between both actuators.
                    ver_total = abs(active['hr']) + abs(passive_ver)
                    # Divide the horizontal distance to advance proportional to
                    # both heights.
                    k = abs(passive_ver) / ver_total
                    hor = k * active['wr']
                    # Now, take the passive actuator to the ground.
                    ver = passive_ver
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

        return index, hor, ver, re_res['end'] and fr_res['end']

    def set_to_ground(self):
        """Compute the distance to place both wheels on the ground.

        Return:
          - Index of the wheel to shift:
            - 0: Rear wheel.
            - 1: Front wheel.
          - Vertical height to shift the actuator.

        However, if both wheels are in the ground, return None as first value.
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
        """Return the shift of an actuator.

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
