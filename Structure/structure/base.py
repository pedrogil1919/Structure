"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Definition of all the elements which composes the structure:
- Actuator (x4).
- Current position with respect to the environment.
- Current elevation with respect to the ground.
"""

from math import asin, sqrt
from enum import Enum

# NOTE: Sometimes opencv changes the data type for drawing function. So it is
# better to import the correct data type this way.
from numpy import int as cv_datatype
import cv2

from structure.actuator import WheelActuator
from structure.pair import ActuatorPair
from simulator.error_distance import InclinationError, StructureError
from physics.wheel_state import MAX_GAP


# State of the structure acording to its maximum inclination.
class StructureState(Enum):
    """Possible states for an actuator."""
    InclinationNormal = 1
    InclinationLimit = 2
    InclinationExit = 3


HOR_MARGIN = 0.0
VER_MARGIN = 0.0


class ConfigurationError(ValueError):
    pass


class Pose():
    """Class to store the current position of the structure.

    """

    def __init__(self, horizontal, vertical, inclination,
                 width=0.0, internal=0.0):
        """Initial position:

        Arguments:
        horizontal -- Horizontal position
        vertical -- Vertical position
        inclination -- Inclinat. as the difference between front and rear part
        width -- Width of the structure.
        internal -- Relative horizontal position of the internal actuators.

        """
        self.__horizontal = horizontal
        self.__vertical = vertical
        self.__inclination = inclination
        # Note that the width is a constant.
        self.__WIDTH = width
        self.__INTERNAL = internal

    def get_horizontal(self):
        return self.__horizontal

    def get_vertical(self):
        return self.__vertical

    def get_inclination(self):
        return self.__inclination

    def get_angle(self):
        return asin(self.__inclination / self.__WIDTH)

    def get_width(self):
        return self.__WIDTH

    def get_internal(self):
        return self.__INTERNAL

    # NOTE: The properties of the object can not be modified directly, but
    # only by adding a new value to the current one.
    def add_horizontal(self, value):
        self.__horizontal += value

    def add_vertical(self, value):
        self.__vertical += value

    def add_inclination(self, value):
        self.__inclination += value

    def __sub__(self, prev):

        h = self.horizontal - prev.horizontal
        i = self.inclination - prev.inclination
        v = self.vertical - prev.vertical
        return Pose(h, v, i)

    horizontal = property(get_horizontal, None, None, None)
    vertical = property(get_vertical, None, None, None)
    inclination = property(get_inclination, None, None, None)
    angle = property(get_angle, None, None, None)
    WIDTH = property(get_width, None, None, None)
    INTERNAL = property(get_internal, None, None, None)


class Base:
    """Class to the define the whole structure."""

    def __init__(self, size, wheels, stairs, limits=None, debug=None):
        """Constructor:

        Arguments:
        size -- Dictionary with the dimensions of the structure (see paper):
          - a, d, c: Partial dimensions of the base.
          - d: Height of the structure (and length of actuators).
          - m: Margin
          - g: Gap between floor and lower base.
        wheels -- Dictionary with the radius of wheels:
          - r1, r2, r3, r4: Wheels radius (can be different).
        stairs -- Physical structure of the stairs.
        limits -- Aditional constrains for the mechanical dimensions. It must
            be a callable functions, whose arguments are the three dictionaries
            defined above, and raises a ValueError if not fulfill the
            constrains.
        debug -- Debug option. If given, can be used in any point to draw
            the current state of the structure. It is a dictionary with the
            keys:
                - graphics: Graphics module.
                - simulator: Simulator module.

        Raises a ConfigurationError(ValueError) exception if the dimensions
        does not allow to build a valid structure.

        """
        # Check whether the dimensions given fulfill the restrictions.
        try:
            limits(size, wheels)
        except ValueError:
            raise ConfigurationError(
                "Dimensions does not fulfill mechanical restrictions")
        except TypeError:
            pass

        # Maximum inclination angle. It is computed 6from both, the structure
        # dimensions, which gives the maximum inclination allowed before the
        # wheel collide among them, and the maximum inclination given by the
        # user, which should be computed from the range of the L9 actuator.
        self.MAX_INCLINE = self.max_inclination(size, wheels)

        if debug is not None:
            self.DEBUG = debug
        self.STAIRS = stairs
        # Main distances of the structure.
        a = size['a']
        b = size['b']
        c = size['c']
        d = size['d']
        # NOTE: The distance g has nothing to do with the control module. It is
        # just for representation purposes.
        g = size['g']

        # When computing the distance for a wheel to move, sometimes we need to
        # give a small margin to prevent the wheel to collide with the stair.
        # This is the meaning of these margins.
        margins = (size['h'], size['v'])

        r1 = wheels['r1']
        r2 = wheels['r2']
        r3 = wheels['r3']
        r4 = wheels['r4']
        # Check dimensions:
        # Restriction 1: The separation between actuators must be greater than
        # the wheel radius, so that
        # Size of the structure.
        self.LENGTH = d + g
        # Size of the actuators.
        self.HEIGHT = d

        # Current position of the structure.
        w = a + b + c
        internal = (0.0, a / w, (a + b) / w, 1.0)
        self.position = Pose(0.0, d + g, 0.0, w, internal)

        # Create array of actuators.
        # NOTE: The total length of an actuator is equal to the height of the
        # structure (d), plus the gap between the floor and the lower base of
        # the structure (g), minus the wheel radius (r).
        actuator1 = WheelActuator(
            0, d, d + g - r1, r1, self.position, stairs, margins)
        actuator2 = WheelActuator(
            a, d, d + g - r2, r2, self.position, stairs, margins)
        self.REAR = ActuatorPair(actuator1, actuator2)
        actuator3 = WheelActuator(
            a + b, d, d + g - r3, r3, self.position, stairs, margins)
        actuator4 = WheelActuator(
            a + b + c, d, d + g - r4, r4, self.position, stairs, margins)
        self.FRNT = ActuatorPair(actuator3, actuator4)

        # Set the state of the structure to normal, since the initial
        # inclination is 0, so the structure is not on it inclination limit.
        self.state = StructureState.InclinationNormal

    def reset_position(self):
        """Place the structure in the initial position.

        """
        # Set position to the origin of the stair.
        self.elevation = self.LENGTH
        self.position = 0.0
        self.angle = 0.0
        # Set the actuators to its original position (0).
        re = self.REAR.get_actuator_position(0)
        fr = self.REAR.get_actuator_position(1)
        self.REAR.shift_actuator(-re, -fr, 0.0)
        re = self.FRNT.get_actuator_position(0)
        fr = self.FRNT.get_actuator_position(1)
        self.FRNT.shift_actuator(-re, -fr, 0.0)
        # Check that everything is ok.
        col, stb = self.check_position()
        col.add_stability(stb)
        return col

    @classmethod
    def max_inclination(cls,  size, wheels):
        # Check that all values are positive:
        for dim in size.values():
            if dim < 0:
                raise ConfigurationError(
                    "Structure dimensions must be positive values.")
        for dim in wheels.values():
            if dim < 0:
                raise ConfigurationError(
                    "Wheel radius must be positive values.")
        try:
            length = size['a'] + size['b'] + size['c']
            h1 = sqrt(size['a']**2 - (wheels['r1'] + wheels['r2'])**2)
            h1 *= length / size['a']
            h2 = sqrt(size['b']**2 - (wheels['r2'] + wheels['r3'])**2)
            h2 *= length / size['b']
            h3 = sqrt(size['c']**2 - (wheels['r3'] + wheels['r4'])**2)
            h3 *= length / size['c']
            h4 = size['n']
        except ValueError:
            raise ConfigurationError(
                "The dimensions of the structure are incorrect")
        return min([h1, h2, h3, h4])
    ###########################################################################
    # MOTION FUNCTIONS
    ###########################################################################
    # Note for all motion functions:
    # When a given motion can not be completed for any cause (wheel collision
    # or wheel pair unstable), the corresponding function does not perform the
    # required motion, and returns the error distance, that is, if we call
    # the same function again subtracting the distance returned, the motion
    # now can be completed, and the structure will be set at the limit. In
    # fact, the distance returned is of the opposite sign, so that, the correct
    # distance will be the required distance plus the distance returned by the
    # function.

    def check_position(self, margin=False):
        """General function to check the validity of the current position.

        After any structure motion, the position of the structure MUST be
        checked, since the functions performing the motion do not do any check.
        For this reason, this function must be called after any motion.
        The function returns two dictionaries with the following keys:
          - Dictionary 1 (for wheel collisions or actuators reaching the
            further than either limit):
              - res: If True, the position is valid in terms of wheel or
                actuator
              - collisions. In this case, only this key exists in the
                dictionary. In case of False, include these other keys:
              - ver: Vertical distance for the error. That is, if we call
                again the same function adding the error returned in this key
                to the original distance, the function will succeed.
              - hor: Similar to vertical, but for horizontal distances.
              - act: When both the wheel collides vertically and the actuator
                reaches its lower bound, the key 'ver' includes the largest of
                both distances. However, for some motions only the actuator
                error is needed. This value in returned in this key.
          - Dictionary 2 (for wheel pairs geting to an unstable position).
              - res: If True, the position is valid in terms of wheel stability
                (at least one of the wheels of the pair is in a stable
                position).
                In this case, only this key exists in the dictionary. In case
                of False, include these other keys:
              - dis: horizontal distance needed to advance the structure so
                that the one of the wheel pair is place back to a stable
                position.

        Arguments:
        margin -- See WheelActuator.check_actuator function.
        """
        # Check if any wheel has collided with the stairs.
        re_re, re_fr, re_pair = self.REAR.check_collision(margin)
        fr_re, fr_fr, fr_pair = self.FRNT.check_collision(margin)

        # Check for possible collisions due to inclinations.

        # Check if the maximum inclination (positive or negative) has been
        # reached.
        # Get differences in height between rear and front actuators.
        __, y0 = self.REAR.REAR.JOINT.position(0)
        __, y3 = self.FRNT.FRNT.JOINT.position(0)
        # And check if the maximum distance has been reached. Note that this
        # check must be done in both directions.
        # If the limit has been reached, include this value in the collision
        # object.
        if y0 - y3 > self.MAX_INCLINE + MAX_GAP:
            error = y0 - y3 - self.MAX_INCLINE
        elif y3 - y0 > self.MAX_INCLINE + MAX_GAP:
            error = y0 - y3 + self.MAX_INCLINE
        else:
            error = None
        inclination = InclinationError(error)

        actuators = (re_re, re_fr, fr_re, fr_fr)
        pairs = (re_pair, fr_pair)
        return StructureError(actuators, pairs, inclination)

    def advance(self, distance, check=True):
        """Advance the structure horizontally.

        Arguments:
        distance -- Horizontal distance to move (positive, move right).
        check -- If True, after performing the motion, check that the structure
          is still in a valid position. A False value is intended to place
          the structure back to a valid position after a wrong motion inside
          the own function.

        Return (see note above).
        """
        # Get previous position for speed computation.
        self.prev_pos = self.position
        # Update structure position
        self.position.add_horizontal(distance)
        if not check:
            # This option is called inside this function, so do not need to
            # return any value as long as we take this into account bellow,
            # where the function is called with check set to False.
            return

        # From here on, check the validity of the motion.
        structure_position = self.check_position()
        if structure_position:
            return structure_position
        # Set the structure back to its original position.
        # NOTE: When check is set to False, the function does not return any
        # value, so leave the call without receiving any value.
        self.advance(-distance, False)
        # Check that everything is OK again.
        if self.check_position():
            return structure_position
        # If we place the structure back to its original position, there should
        # not be any error. If this error happens, it is a run time error.
        raise RuntimeError("Error in advance structure")

    def elevate(self, height, wheel=None, check=True, margin=True):
        """Elevate (or take down) the whole structure.

        Arguments:
        height -- Vertical distance to move (positive, structure move
            upwards.
        wheel -- List of four elements, each one can be either None of float:
            If None, shift the corresponding actuator a height so that the
            wheel remains in the same position. For example, if the wheel is
            on the ground and this value is None, after the elevation, the
            wheel is still on the ground. If float, this actuator must be
            shifted the distance given for that actuator.
        check -- See advance function.
        margin -- See check_position function.

        Return (see note above).
        """
        if wheel is None:
            wheel = 4 * [None]
        # Elevate the structure,
        self.position.add_vertical(height)
        # and place the actuators in the correct position.
        self.REAR.shift_actuator(wheel[0], wheel[1], height)
        self.FRNT.shift_actuator(wheel[2], wheel[3], height)

        if not check:
            # See comment in advance function.
            return

        # Check if any of the actuators has reached one of its bounds.
        structure_position = self.check_position(margin)
        if structure_position:
            # Everything is OK.
            return structure_position

        # Leave the structure in its original position.
        # For the actuator motion, we have to change also the sign of the
        # motion. Since some of tha values are None, we have to change only
        # the not None values.
        wheel_aux = [-w if (w is not None) else w for w in wheel]
        self.elevate(-height, wheel_aux, False)
        # Check that everything is OK again.
        # NOTE: In this case, never a stability error can happen, and so, we
        # need not collect the stability error.
        if self.check_position(margin):
            return structure_position

        raise RuntimeError("Error in elevate")

    def incline(self, height, wheel=None, fixed=0, check=True, margin=True):
        """Incline the base of the structure.

        Arguments:
        height -- Vertical distance to move the exterior actuators (actuators
            0 or 3). The angle can be computed from this value and the length
            of the structure.
        wheel -- See elevate function.
        fixed -- When rotating the structure, the point that remains fixed,
            that must be one of the joint of any of the actuators. For
            instance, if 0, the structure incline fixing the rear corner, and
            so elevating the front corner.
        check -- See advance function.
        margin -- see advance function.

        Return (see note above).
        """

        # Get vertical coordinates of the outer joints to update structure
        # angle.
        current_inclination = self.position.inclination
        try:
            # Update the angle taking into account the new height to lift.
            self.position.add_inclination(height)
            # self.angle = asin((h + height) / self.WIDTH)
        except ValueError:
            # In case we pretend to elevate a height larger than the maximum
            # allowed, that is, in the previous instruction we try to compute
            # the arcsin of a value greater than 1, an error is raised. We
            # need to return false.
            # Note that, although this instruction also raises an error because
            # it overpass the maximum inclination, this error is raised before,
            # and so, it must be detected here.
            if current_inclination + height > 0:
                raise InclinationError(+self.MAX_INCLINE -
                                       current_inclination - height)
            else:
                raise InclinationError(-self.MAX_INCLINE -
                                       current_inclination - height)

        if wheel is None:
            wheel = 4 * [None]

        # Current computations keep fixed the rear edge of the structure. To
        # change this to leave fixed one of the other actuators, we have to
        # elevate the structure in the opposite direction.
        if fixed == 0:
            prop_height = self.REAR.REAR.JOINT.proportional_lift(height)
        elif fixed == 1:
            prop_height = self.REAR.FRNT.JOINT.proportional_lift(height)
        elif fixed == 2:
            prop_height = self.FRNT.REAR.JOINT.proportional_lift(height)
        elif fixed == 3:
            prop_height = self.FRNT.FRNT.JOINT.proportional_lift(height)

        self.position.add_vertical(-prop_height)
        # Set the actuators to its new position before inclining. Note
        # that we need not check whether they are in a valid position
        # since it can happen that, even in an invalid position at this
        # step, the actuator can return back to a valid position after
        # the inclination.
        # For the actuator to position independently, in this previous
        # step, we need to fix it to the elevation of the structure, as
        # opposed of the rest, that need to be shifted so that the wheels
        # remains in the same position.
        wheel_aux = [0 if (w is not None) else None for w in wheel]
        self.REAR.shift_actuator(wheel_aux[0], wheel_aux[1], -prop_height)
        self.FRNT.shift_actuator(wheel_aux[2], wheel_aux[3], -prop_height)

        # Check inclination state:
        new_inclination = abs(current_inclination + height)
        if new_inclination < self.MAX_INCLINE - MAX_GAP:
            self.state = StructureState.InclinationNormal
        elif new_inclination < self.MAX_INCLINE + MAX_GAP:
            self.state = StructureState.InclinationLimit
        else:
            self.state = StructureState.InclinationExit

#        # If we fix the rear wheel, the structure does not move (that is, the
#        # reference frame does not move).
#        # However, if we fix the front wheel, the reference frame does move,
#        # and so, we need to compute that motion to leave the front wheel
#        # fixed.
#         x3d = 0.0
#         if fix_front:
#             # Compute the motion undergone by the front wheel.
#             __, __, x3d, __ = self.FRNT.position(0)
#             x3d -= x3
#             # And move the structure in the opposite direction.
#             self.position -= x3d

        # Elevate all the actuators (except the first one that does not move)
        # the corresponding distance to get the required inclination.
        self.REAR.shift_actuator_proportional(wheel[0], wheel[1], height)
        self.FRNT.shift_actuator_proportional(wheel[2], wheel[3], height)

        if not check:
            return

        # Check the validity of the motion.
        # TODO: When fixing front or elevating the rear wheel, it is possible
        # that this function work wrongly. Check it.
        structure_position = self.check_position(margin)

        if structure_position:
            return structure_position

        # Leave the structure in its original position.
        wheel_aux = [-w if (w is not None) else w for w in wheel]
        self.incline(-height, wheel_aux, fixed, False)
        # Check that everything is OK again.
        if self.check_position(margin):
            return structure_position

        raise RuntimeError("Error in incline function")

    def shift_actuator(self, index, height, check=True, margin=True):
        """Shift one actuator independently.

        Arguments:
        index -- Index of actuator (0-3)
        height -- Distance to shift (positive, move downwards).
        check -- See advance function.
        margin -- see advance function.

        Return (see note above).
        """
        # Select the actuator to shift.
        if index == 0:
            self.REAR.shift_actuator(None, 0.0, height)
        elif index == 1:
            self.REAR.shift_actuator(0.0, None, height)
        elif index == 2:
            self.FRNT.shift_actuator(None, 0.0, height)
        elif index == 3:
            self.FRNT.shift_actuator(0.0, None, height)

        if not check:
            return

        # Check if the actuator has reached one of its bounds.
        structure_position = self.check_position(margin)

        # The variable col is for possible collisions with the steps, or an
        # actuator reaching one of its bounds.
        # The variable stb is for checking that, after elevating one wheel,
        # the other wheel of the pair is still in a stable position.
        if structure_position:
            return structure_position

        # Leave the actuator in its original position.
        self.shift_actuator(index, -height, False)
        # Check that everything is OK again.
        if self.check_position(margin):
            return structure_position
        raise RuntimeError("Error in shift actuator.")

###############################################################################
###############################################################################
    def incline_and_advance(self, height, wheel=None,
                            fixed=0, check=True, margin=True):
        """Incline structure, and advance if a collision happens.

        This function is similar to incline, but if after the inclination any
        wheel has collided with the stair, or a pair is in an unstable
        position, advance the structure to place it in a valid position.

        return the structure state.

        """
        # Incline the structure.
        state1 = self.incline(height, wheel, fixed, check, margin)
        if state1:
            # Check if there is a error when inclining.
            return state1

        # Check if there is a horizontal collision when inclining.
        advance = state1.horizontal()
        state2 = self.advance(advance)
        if not state2:
            # This happens when one wheel is close to one edge, and the other
            # is close to a step, so that when inclining in one direction one
            # of the wheel collides, and when trying to correct this collision,
            # the other wheel collides in the opposite direction.
            advance += state2.horizontal()
            state3 = self.incline(height, wheel, fixed, check, margin)
            return state3

        state3 = self.incline(height, wheel, fixed, check, margin)
        return state3

    def allowed_inclination(self, height):
        """Check if the structure can be inclined the given height.

        Return hte maximum allowed inclination.

        """
        # Get differences in height between rear and front actuators.
        __, y0 = self.REAR.REAR.JOINT.position(0)
        __, y3 = self.FRNT.FRNT.JOINT.position(0)
        current_inclination = y3 - y0
        next_inclination = current_inclination + height
        if next_inclination > self.MAX_INCLINE:
            return +self.MAX_INCLINE - current_inclination, \
                next_inclination - self.MAX_INCLINE
        if next_inclination < -self.MAX_INCLINE:
            return -self.MAX_INCLINE - current_inclination, \
                next_inclination + self.MAX_INCLINE
        return height, 0.0

    def make_room_wheelN(self, actuator, height):
        """Generate aditional vertical space for actuator 0, 1 or 2.

        Arguments:
        actuator -- index of the actuator which is actually pushing the
            structure.
        height -- distance the actuator has collided with the structure, and
            so, is the space we need to make, to allow the actuator to shift
            the required distance.

        In this case, the first option is elevate the structure to gain enough
        space to shift the actuator. If not possible this can be due to:
            - the structure collides with another actuator. In this case, we
              can try to incline fixing the colliding actuator to gain more
              space.
            - The structure has reached its limit. In this case, the motion is
              not possible.

        Returns the structure state, the advance, inclination and elevation
        given to the structure.
        """
        # Try to elevate the required distance.
        state1 = self.elevate(height, margin=False)
        if state1:
            # If success, the actuator can alreay be shifted.
            return True
        # If not possible, first take the structure to the maximum height
        # possible.
        # Get the height that the structure has collided with one of the
        # actuators (the actuator can be any but the current actuator, since
        # this is the actuator that is pushing the structure).
        height += state1.elevation()
        # And elevate the structure this height to place the structure in its
        # limit.
        if not self.elevate(height, margin=False):
            raise RuntimeError

        # Check the actuator that has actually collided with the structure.
        # NOTE: Although more than one actuator can have collided, we choose
        # the one that have collided the most, taking into account that we need
        # to push the actuator given. This is considered in both following
        # functions.
        col_actuator = state1.colliding_actuator(actuator)
        col_incline = state1.inclination(actuator)
        # Check if the required inclination is greater than the maximum
        # allowed.
        col_incline, front_elevate = self.allowed_inclination(col_incline)

        # Try to incline fixing the actuator that has actually collided the
        # most.
        state2 = self.incline_and_advance(col_incline,
                                          fixed=col_actuator,
                                          margin=False)
        if state2:
            if front_elevate != 0:
                return False
            ini_actuator = state1.colliding_actuator()
            return (col_actuator == ini_actuator)
        # In this case, the structure has collided with another actuator in
        # the opposite direction.
        col_incline += state2.inclination(col_actuator)
        state3 = self.incline_and_advance(col_incline,
                                          fixed=col_actuator, margin=False)
        if not state3:
            raise RuntimeError
        return False

    def make_room_wheel3(self, height):
        """Generate aditional vertical space for actuator 3.

        Arguments:
        height -- distance the actuator has collided with the structure,
            and so, is the space we need to make to allow the actuator to shift
            the required distance.

        In this case, the first option is incline the structure from the front
        to gain enough space to shift the actuator. If not possible this can
        be due to:
            - the structure colliding with the internal actuators. In this case
              we need to incline the structure from the rear in the opposite
              direction.
            - the structure has reached its limit. In this case, the motion is
              not possible.

        """
        # Check if the required inclination is greater than the maximum
        # structure inclination.
        # In case it is true, divide the motion into a inclination (the maximum
        # possible) plus an elevation for the rest of the height.
        front_incline, front_elevate = self.allowed_inclination(height)

        # Incline the required (or maximum) height.
        state1 = self.incline_and_advance(front_incline, margin=False)
        if state1:
            # Check whether we also need to elevate the structure.
            state2 = self.elevate(front_elevate, margin=False)
            if not state2:
                # If we can not elevate the whole distance, elevate just the
                # distance possible.
                front_elevate += state2.elevation()
                if not self.elevate(front_elevate, margin=False):
                    raise RuntimeError
                # In this case, no error is raised but the structure can not
                # complete the whole motion, so that return false.
                return False
            # In this case, the whole distance has been completed, and so,
            # return true.
            return True

        # If we can not incline the distance required, it is due to an
        # actuator collision in the direction of motion.
        # Get the structure until it collided with the actuator.
        front_incline += state1.inclination(0)
        if not self.incline_and_advance(front_incline, margin=False):
            raise RuntimeError
        # Detect the colliding actuator.
        col_actuator = state1.colliding_actuator(3)
        # TODO: If the structure collides with the front actuator, this is due
        # to a shift greater than the structure height. In this case, it is
        # possible to make a greater distance, but we need to edit this if.
        # See test_make_room_wheel3 -> motion7t to see an example of this.
        # However, I think it is better to prevent the control module to
        # perform a shift greater than the structure height. In any case, this
        # function do its best and do not raise any exception.
        if col_actuator == 3:
            return False
        # And compute the distance that the structure must incline fixing the
        # colliding actuator to get the distance required.
        rear_height = state1.inclination(3) - state1.inclination(0)
        # Try if we can get the required distance inclining fixing the
        # colliding actuator.
        # NOTE: We can not capture the elevation height (in case we reach the
        # maximum inclination), since the structure is already touching the
        # colliding actuator, an no more height can be gain. In case we needed
        # more distance, the complete motion can not be reached.
        rear_incline, rear_elevate = self.allowed_inclination(rear_height)
        state2 = self.incline_and_advance(rear_incline,
                                          fixed=col_actuator, margin=False)
        if state2:
            # Check if we also need to elevate.
            # NOTE: This function is not intended to elevate the structure,
            # since the structure is already colliding with the actuator. This
            # is only intended to check whether the whole distance has been
            # completed or not.
            total_elevate = rear_elevate + front_elevate
            if self.elevate(total_elevate, margin=False):
                # When the structure is close to the limit of the central
                # actuators, it is possible that the structure collides with
                # actuator 1 when inclining from the rear, but collides with
                # actuator 2 when inclining from the front. In this case, the
                # function would return True, but the whole motion has not been
                # done, and so, we have to return False.
                ini_actuator = state1.colliding_actuator(0)
                return (col_actuator == ini_actuator)
            else:
                return False
        # In this case, we have collided with another actuator in the
        # opposite direction. Just get the distance we can incline and
        # finish the motion.
        rear_incline += state2.inclination(col_actuator)
        if not self.incline_and_advance(rear_incline,
                                        fixed=col_actuator, margin=False):
            raise RuntimeError
        return False

    def push_actuator(self, index, height, check=True, margin=True):
        """Shift an actuator, and push the structure if not enough room.

        This function is similar to shift actuator, but if the whole motion is
        not possible, try to make more room by elevating - inclining the
        structure. The kind of action is defined as follows:
        - For frontal actuator, first try to incline the structure. If still
          not enough space, try to incline the structure.
        - For the rest of the actuators, first elevate the structure, and if
          not enough space, incline.

        Return:
        - A simulator.StructureError object with the errors raised when
            pushing the actuator.
        - A dictionary with the following keys:
            - If there is initially enough space, an empty dictionary.
            - If theres is not enough space:
                - Advance.
                - Elevate.
                - Incline.
        Note that, even though the shift can be completed or not, the
        previous dictionary is returned always.

        """
        # Try to shift the actuator and cheeck if the motion can be completed.
        state1 = self.shift_actuator(index, height, check, margin)
        if state1:
            return state1

        # Check if the problem is a wheel collision.
        # NOTE: wheel is True only when the collision of the wheel is greater
        # than the collision of the actuator, even if the wheel has had a
        # collision.
        wheel, collision = state1.wheel_collision(index)
        # If the wheel has collided with the stair, the maximum distance we
        # can take the wheel down is equal to the initial height minus the
        # distance the wheel has collided. In case there is no collision, this
        # instruction do nothing.
        height += collision
        if wheel:
            # In this case, the problem is that the wheel has collided with
            # the stair, and so, we only can take the wheel down to the
            # stair and finish (we do not need to make more room in the
            # sctructure, since the wheel collides before the actuator).
            if not self.shift_actuator(index, height, check, margin):
                raise RuntimeError
            # State1 keeps the total distance the actuator can not move, so
            # return this value.
            return state1

        # However, since there is also an actuator collision, trying to take
        # the actuator down will raise the same collision. We use this
        # collision to compute the distance we have to make room with the
        # following functions.
        state2 = self.shift_actuator(index, height, check, margin)
        if state2:
            raise RuntimeError
        # If not possible, just shift the distance that is actually possible.
        distance = state2.elevation()
        height += distance
        if not self.shift_actuator(index, height, check, margin):
            raise RuntimeError

        # And try to make more space by elevating / inclinating the structure.
        if index == 3:
            res = self.make_room_wheel3(distance)
        elif index == 2:
            res = self.make_room_wheelN(2, distance)
        elif index == 1:
            res = self.make_room_wheelN(1, distance)
        elif index == 0:
            res = self.make_room_wheelN(0, distance)

        if res:
            # If the previous function can make enough space, trying to shift
            # the actuator should not cause any error.

            if not self.shift_actuator(index, -distance, check, margin):
                raise RuntimeError
            # Finally, check if there were any wheel collision. In this case,
            # the motion should raise a colllision, but we need it to return
            # this collision to the calling function.
            state4 = self.shift_actuator(index, -collision, check, margin)
            return state4

        # In case the structure can not make enough space for the shift, so,
        # perform the motion just to generate the error object to return.
        # Check again if the motion is possible.
        height = -distance - collision
        state4 = self.shift_actuator(index, height, check, margin)
        if state4:
            raise RuntimeError
        height += state4.elevation()
        if not self.shift_actuator(index, height, check, margin):
            raise RuntimeError
        return state4

    def get_motion(self, prev_structure):
        """Computes the motion between current structure and previous one.

        Return the horizontal motion, elevation and inclination between this
        structure and the structure given as an argument.

        """
        return self.position - prev_structure.position

    # =========================================================================
    # Control functions.
    # =========================================================================

    def get_wheels_distances(self):
        """Compute the distances from a wheel to the stairs.

        Return:
          - The index of the wheel to shift, that is, the wheel that is closest
            to its corresponding step:
              - 0: Rearmost wheel.
              - 1:
              - 2:
              - 3: Frontmost wheel.
          - The horizontal distance to move.
          - The vertical height for the wheel to shift.
          - The index of the wheel, in the other pair, that is closest to its
            corresponding step.
          - The vertical height for this second wheel.

        See stair.set_distances function, and getDistances.svg.
        """
        # Compute distances for the rear pair, and the front pair.
        re_id, re_hor, re_ver, re_end = self.REAR.get_wheel_distances()
        fr_id, fr_hor, fr_ver, fr_end = self.FRNT.get_wheel_distances()
        # If the front pair has reached the end of the stair, but not the rear
        # pair.
        if fr_end and fr_ver < -MAX_GAP:
            # And also, one of the wheels is not still in the ground, take the
            # front wheel right to the ground.
            return fr_id + 2, re_hor / 2, fr_ver, re_id, re_hor, re_ver, re_end

        # Take the minimum of both pairs.
        if re_hor < fr_hor:
            # Just for the beginning of stair, it is better not to lift the
            # second wheel when upstairs. If we do nothing else, the second
            # wheel certainly will lift, which is not the desired behabior. For
            # that reason, only when a wheel is not close enough to the stair,
            # this will not be lift. The reference is a quarter of the width of
            # the structure.
            # if re_hor > self.WIDTH / 4:
            #     fr_ver = 0.0
            # TODO: Review this comment.
            # When the structure is far apart from the stair, it is better to
            # advance horizontaly until the structure is close enough. This is
            # done with this code.
            # if re_hor > self.WIDTH / 4:
            #     re_ver = 0.0
            #     re_hor -= self.WIDTH / 4
            return re_id, re_hor, re_ver, fr_id + 2, fr_hor, fr_ver, re_end
        else:
            # Same comment that above.
            # if re_hor > self.WIDTH / 4:
            #     re_ver = 0.0
            # NOTE: The index of wheel are numbered from 0. Since the rear
            # wheel is the third wheel of the structure, we have to add 2 to
            # the index returned for the pair.
            return fr_id + 2, fr_hor, fr_ver, re_id, re_hor, re_ver, re_end

    def set_horizontal(self):
        """Return the distance to place each wheel on the ground ."""
        # Check if also any wheel need to be set to the ground.
        re_res = self.REAR.set_to_ground()
        fr_res = self.FRNT.set_to_ground()

        return re_res, fr_res

    def get_actuator_position(self, index):
        """Return the current position of the actuators.

        Returns the position of a given actuator.

        Parameters:
        index -- Index of the actuator.
          - 0: Rearmost actuator.
          - 1
          - 2
          - 3: Frontnost actuator.
        """
        if index == 0:
            return self.REAR.get_actuator_position(0)
        elif index == 1:
            return self.REAR.get_actuator_position(1)
        elif index == 2:
            return self.FRNT.get_actuator_position(0)
        elif index == 3:
            return self.FRNT.get_actuator_position(1)
        elif index == 9:
            return self.get_actuator_L9()
        raise ValueError("Actuator index are 0 - 3.")

    def get_inclination(self):
        """Returns the inclination of the structure."""
        # The inclination is computed as the difference in height between the
        # front and the rear joints of the structure.
        __, y0, __, __ = self.REAR.position(0)
        __, __, __, y3 = self.FRNT.position(0)
        return y3 - y0

    # @staticmethod
    def get_actuator_L9(self):
        """Translate the structure inclination to actuator L9 position"""
        return self.get_inclination()
        # d = 98.00
        # LH = 398.00
        # m = 30.58
        # n = 36.14
        # L9 = sqrt(d**2 + LH**2 - 2 * d * self.get_inclination() - n**2) - m
        # return L9

    def get_elevation(self):
        """Return the elevation of the structure."""
        # The elevation of the structure is the same as the position of the
        # rear joint.
        return self.REAR.REAR.d

    def get_speed(self):
        """Return the speed of the strucure."""
        # TODO: Update function with the dynamics of the structure.
        # current_position = Pose(self.)
        return self.current_pos - self.prev_pos

    # def get_acceleration(self):
    #     """Return the acceleration of the strucure."""
    #     # TODO: Update function with the dynamics of the structure.
    #     return 0.0

    def actuator_positions(self):
        """Return the current positions of the actuators.

        This function is designed for representation purposes.

        """
        pos = [
            self.get_actuator_position(0),
            self.get_actuator_position(1),
            self.get_actuator_position(2),
            self.get_actuator_position(3),
            self.get_actuator_position(9)
            # self.get_inclination()
            # ,
            # self.get_speed()
        ]
        return pos

    def wheel_positions(self):
        """Return the horizontal position of the center of the wheels.

        This function is designed for representation purposes.

        """
        x1, __, x2, __ = self.REAR.position(0)
        x3, __, x4, __ = self.FRNT.position(0)
        return (x1, x2, x3, x4)
    # =========================================================================
    # Drawing functions.
    # =========================================================================
    # Base colors and widths.
    BASE_COLOR_NORMAL = (0xFF, 0x00, 0xB3)
    BASE_COLOR_LIMIT = (0x00, 0xFF, 0xB3)
    BASE_COLOR_EXIT = (0x00, 0x00, 0xFF)
    BASE_WIDTH = 6

    def draw(self, origin, image, scale, shift):
        """Draw complete wheelchair."""
        x1, y1, __, __ = self.REAR.position(0)
        __, __, x2, y2 = self.FRNT.position(0)

        # Change the color of the structure acording to the MAX_INClINATION
        if self.state == StructureState.InclinationNormal:
            color_base = self.BASE_COLOR_NORMAL
        elif self.state == StructureState.InclinationLimit:
            color_base = self.BASE_COLOR_LIMIT
        else:
            color_base = self.BASE_COLOR_EXIT

        cx1 = cv_datatype(scale * (origin[0] + x1))
        cy1 = cv_datatype(scale * (origin[1] - y1))
        cx2 = cv_datatype(scale * (origin[0] + x2))
        cy2 = cv_datatype(scale * (origin[1] - y2))
        cv2.line(image, (cx1, cy1), (cx2, cy2), color_base,
                 self.BASE_WIDTH, cv2.LINE_AA, shift)

        dy1 = cv_datatype(scale * (origin[1] - y1 + self.HEIGHT))
        dy2 = cv_datatype(scale * (origin[1] - y2 + self.HEIGHT))
        cv2.line(image, (cx1, dy1), (cx2, dy2), color_base,
                 self.BASE_WIDTH, cv2.LINE_AA, shift)

        chair_position = [origin[0] + (x1 + x2) / 2,
                          origin[1] - (y1 + y2) / 2]
        self.draw_chair(chair_position, image, scale, shift)
        self.REAR.draw(origin, image, scale, shift)
        self.FRNT.draw(origin, image, scale, shift)

    # Chair colors and width (witdh en pixels)
    CHAIR_ELEVATION = 10
    CHAIR_SHIFT = 10
    CHAIR_HEIGHT = 400
    CHAIR_BASE = 300
    CHAIR_BACK = 40
    CHAIR_COLOR = (0x40, 0x50, 0xA0)
    CHAIR_WIDTH = 15

    def draw_chair(self, position, image, scale, shift):
        position[1] -= self.CHAIR_ELEVATION
        position[0] -= self.CHAIR_SHIFT
        x1 = cv_datatype(scale * position[0] + self.CHAIR_BASE)
        y1 = cv_datatype(scale * position[1])
        x2 = cv_datatype(scale * position[0])
        y2 = cv_datatype(scale * position[1])
        x3 = cv_datatype(scale * position[0] - self.CHAIR_BACK)
        y3 = cv_datatype(scale * position[1] - self.CHAIR_HEIGHT)
        cv2.line(image, (x1, y1), (x2, y2), self.CHAIR_COLOR,
                 self.CHAIR_WIDTH, cv2.LINE_AA, shift)
        cv2.line(image, (x2, y2), (x3, y3), self.CHAIR_COLOR,
                 self.CHAIR_WIDTH, cv2.LINE_AA, shift)

    def draw_wheel_trajectory(self, origin, image, scale, shift, index):
        """Draw the trajectory of the center of the wheel."""
        if index == 0 or index == 1:
            self.REAR.draw_trajectory(origin, image, scale, shift, index)
        if index == 2 or index == 3:
            self.REAR.draw_trajectory(origin, image, scale, shift, index - 2)

###############################################################################
# End of file.
###############################################################################
