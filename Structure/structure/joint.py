"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Definition of the part of the structure that fix the actuator to the main base.
This module implements the trigonometric computation to calculate the position
of an actuator according to the structure inclination.
"""

from math import cos, sin


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

###############################################################################
# End of file.
###############################################################################
