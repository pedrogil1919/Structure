"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Definition of the part of the structure that fix the actuator to the main base.
This module implements the trigonometric computation to calculate the position
of an actuator according to the structure inclination.
"""

from math import cos, sin, sqrt

from physics.wheel_state import MAX_GAP


class Joint:
    """Define the joint of an actuator with the structure."""

    def __init__(self, structure_position, position):
        """Constructor:

        Arguments:
        structure_position -- Reference to the actual position of the base that
          hold the joint. Used to compute absolute coordinates from the offset
          of the joint with respect to the base origin.
        relative_position -- Horizontal distance of the actuator to the back
            side of the structure with respect to its width.
        """
        self.structure_position = structure_position
        self.relative_position = position / structure_position.WIDTH

    def position(self, height=0):
        """Return the (x, y) position of a given point along the actuator.

        Arguments:
        height -- Vertical distance from the required point to the joint.
        """
        # Get actual coordinates.
        angle = self.structure_position.angle
        x = self.structure_position.horizontal + \
            self.structure_position.WIDTH * self.relative_position * cos(angle)
        y = self.structure_position.vertical - height + \
            self.structure_position.WIDTH * self.relative_position * sin(angle)

        return x, y

    def proportional_lift(self, height):
        """Compute a lift when inclining the structure.

        When the structure inclines, each actuator need to shift a different
        height. This height is proportional to the distance of the actuator
        to the origin of the structure. This function computes the shift for an
        inner actuator when the outer actuator is shift the given distance when
        inclining the structure. This method is only used by the inner
        actuators.

        Arguments:
        height -- Height for the outer actuator, needed to compute the
            proportional height for this actuator.
        """
        return height * self.relative_position

    def inverse_prop_lift(self, height):
        """Compute an inverse height when inclining the structure.

        The function computes the height the exterior actuators need to be
        shifted when one of the inner actuator is shifted the current height
        when inclining the structure. The values returned are:
          - Height for the front actuator.
          - Height for the rear actuator.

        This function can only be called for the interior actuators. For the
        exterior actuators this value will be 0 and infinity.
        """
        fr_height = height / self.relative_position
        re_height = height / (1 - self.relative_position)
        return fr_height, re_height

    def lift_from_horizontal_motion(self, distance):
        """ Computes the inclination height to get a horizontal distance.

        When the structure inclines, all the actuators except one of them
        move horizontally. This function does the opposite, that is, how much
        the structure must incline to get a given horizontal motion for this
        actuator. Return the computed height.
        See lift_from_horizontal_motion.svg.

        Arguments:
        distance -- Horizontal distance
        rear -- If we incline from the rear the motion is different than if
            we incline from the front. For this reason, this argument specifies
            this.

        """
        # Get the current actuator coordinates. The actual height of the
        # actuator is not important. It is only important the difference in
        # height with respect to the rearmost point of the structure.
        d1, h1 = self.position(0.0)
        # The actual coordinates are obtained substracting the actual
        # coordinates of the origin of the structure.
        h1 -= self.base.elevation
        d1 -= self.base.position
        # The distance d2, that is, the position where we want to move the
        # actuator by inclining the structure, is obtaining adding the distance
        # we want to move the actuator.
        d2 = d1 + distance
        # Compute the total height (see lift_from_horizontal_motion.svg).
        try:
            h2 = sqrt(h1**2 + d1**2 - d2**2)
        except ValueError:
            # Note that the width of the structure is equal to sqrt(h1^2+d1^2).
            # If d2 is greater than this value, we are requiring to get a
            # distance greater to the current structure, so this is not
            # possible. In this case, the discriminant becomes negative. This
            # is not possible to happen, but can be due to rounding errors. In
            # this case, set h2 to 0 is the better solution.
            # The other limit is when d2 -> 0. That means that the structure
            # is completely vertical, which is also impossible. Nevertheless,
            # it is not neccesary to control this error, since this should be
            # detected previously.
            h2 = 0.0
        # The value of h2 must be the same than h1.
        # TODO: In the limit (when the structure in completely horizontal),
        # this function can give the opposite sign. Revise, altough in the
        # limit, it is possible that this function in never called.
        if h1 < 0:
            h2 = -h2
        h = h2 - h1
        return h

###############################################################################
# End of file.
###############################################################################
