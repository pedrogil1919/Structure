"""
Created on 15 feb. 2021

@author: pedro.gil@uah.es

Definition of classes to return the list of distance errors when checking the
position of the structure with respect to the stairs.
"""


def greatest(v1, v2):
    """Returns the variable with the greatest absolute value."""
    return v1 if abs(v1) > abs(v2) else v2


def smallest(v1, v2):
    """Return the variable with the smallest absolute value."""
    return v1 if abs(v1) < abs(v2) else v2


def merge_collision(res1, res2):
    """Merge the values of two collision data."""
    # If both variables are True, there is no collision, so return a no
    # collision object.
    if res1 and res2:
        return CollisionErrors()
    # If only res1 is False, all values are from this variable, so return it.
    elif not res1 and res2:
        return res1
    # The same when only res2 is False.
    elif res1 and not res2:
        return res2
    else:
        # If both are false, for each member of the class, choose the greater.
        horizontal = greatest(res1.horizontal, res2.horizontal)
        actuator = greatest(res1.actuator, res2.actuator)
        central = greatest(res1.central, res2.central)
        front = greatest(res1.front, res2.front)
        rear = greatest(res1.rear, res2.rear)
        return CollisionErrors(
            False, horizontal, actuator, central, front, rear)


def merge_stability(res1, res2):
    """Similar than merge_collision, but for stability objects."""
    if res1 and res2:
        return StabilityErrors()
    elif not res1 and res2:
        return res1
    elif res1 and not res2:
        return res2
    else:
        horizontal = greatest(res1.horizontal, res2.horizontal)
        return StabilityErrors(False, horizontal)

###############################################################################
###############################################################################


class CollisionErrors():
    """Class to store all the possible distance errors when checking position.

    After each motion, the structure position must be checked. If there is any
    error (a wheel colliding with a step, an actuator reaching one of it
    bounds, etc), the distance of error must be computed and stores in this
    class.

    """
    def __init__(self, correct=True, horizontal=0.0,
                 actuator=0.0, central=0.0, front=0.0, rear=0.0):
        """Constructor

        Parameters:
        horizontal -- Horizontal distance needed for the wheel which is inside
          a step to place it outside the step, or distance needed to place the
          wheel in a stable position.
        actuator -- Vertical shift needed for an actuator to place the actuator
          in a valid position.
        central -- Vertical distance the structure need to elevate to place
          both the wheels and the actuators in a valid position. Central and
          actuator take the same value except when a wheel is inside the stair
          and the distance the wheel is inside the stair is greater than the
          distance the actuator is out of its lower limit. In any other case,
          both parameters are equal.
        front -- Vertical distance the front actuator need to be shift to place
          the structure in a valid position when inclining the structure (see
          (see inv_proportional_shift.svg).
        rear -- Same as front, for the rear actuator.
        """
        # Save all data.
        self.correct = correct
        self.horizontal = horizontal
        self.actuator = actuator
        self.central = central
        self. front = front
        self.rear = rear

    def __bool__(self):
        return self.correct

    def __str__(self):
        if self.correct:
            return "Correct"
        return f"Error: \n" \
            "Horizontal: %.2f\n" \
            "Actuator: %.2f\n" \
            "Vertical errors:\n" \
            "- Central: %.2f\n" \
            "- Rear actuator: %.2f\n" \
            "- Front actuator: %.2f" % \
            (self.horizontal, self.actuator,
             self.central, self.rear, self.front)

    def add_inclination_errors(self, inclination):
        """Add all inclination errors.

        For some function, they can not compute at the same time the collision
        and the inclination errors. For the calling function to be simpler,
        once the collision data is obtained, use this function to complete the
        data (in case this data need to be included, otherwise leave empty).

        Parameters:
        inclination -- Tupla, with the elements:
          - 0: front inclination error.
          - 1: rear inclination error.
        """
        self.front = inclination[0]
        self.rear = inclination[1]

    def add_stability(self, error):
        """Same as add_inclination_errors, but for stability data."""
        # Only when statibility error is False
        if self and not error:
            self.horizontal = error.horizontal
            self.correct = False
        elif not self and not error:
            self.horizontal = greatest(self.horizontal, error.horizontal)

    def add_inclination_limit(self, value):
        """Add error when the structure reaches its inclination limit.

        The structure has a maximum inclination value, which is computed from
        the structure dimensions (see paper ICINCO). If at any time, when
        inclining the structure, it reaches the maximum value (either possitive
        or negative), a collision error happens and need to be included in the
        collision object.
        """
        # Add this value when it is greater than the front value, that is, when
        # the structure reaches it inclination limit at the same time that an
        # actuator too, set the error to the largest one.
        self.front = greatest(self.front, value)
        self.rear = greatest(self.rear, value)
        self.central = greatest(self.central, value)
        self.correct = False

###############################################################################
###############################################################################


class StabilityErrors():
    """Class to store stability errors

    See CollisionErrors class for more info.

    """
    def __init__(self, correct=True, front=None, rear=None):
        """Constructor:

        Parameters:
        front -- Error distance for the front wheel. If there is no error in
          the front wheel, set the variable to None.
        rear -- Error distance for the rear wheel. Same than front for None.

        """
        if correct:
            self.correct = True
            self.horizontal = 0.0
        elif front is None and rear is None:
            # If both are None, but not correct, means that we have lift both
            # wheels.
            self.correct = False
            self.horizontal = 0.0
        elif front is None and rear is not None:
            # In this case, only know how to set the rear wheel to a stable
            # position. That is, the front wheel is so far apart that we do not
            # know how to set it to a stable position.
            self.correct = False
            self.horizontal = rear
        elif front is not None and rear is None:
            # The same than above, 666666666for the rear wheel.
            self.correct = False
            self.horizontal = front
        else:
            # Both wheels are unstable, and we know how to set both wheels to
            # a stable position. In this case because we only need one wheel
            # stable, choose the smallest value of both distances.
            self.correct = False
            self.horizontal = smallest(front, rear)

    def __bool__(self):
        return self.correct

    def __str__(self):
        if self.correct:
            return "Correct"
        return f"Error: \n" \
            "Horizontal: %.2f\n" % \
            self.horizontal

###############################################################################
# End of file.
###############################################################################
