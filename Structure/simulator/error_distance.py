'''
Created on 20 may. 2022

@author: pedrogil

Definition of classes to return the list of distances errors when checking the
position of the structure with respect to the stairs.
'''
from magic.compat import NONE


class ErrorDistance():
    """
    Generic class. All classes derive from this one.
    """

    # This class save the state of the measures. If every thing is OK, this
    # object sets correct equal True, and does not store any other data.
    def __init__(self, correct=True):
        self.correct = correct

    def __bool__(self):
        return self.correct

    def __str__(self):
        if self.correct:
            return "OK"
        return "Error"


class ActuatorError(ErrorDistance):
    """
    Actuator error:
    In all cases, a value of None means that it is in a valid position. If
    not None, its value indicates the distance that need to move to get to a
    valid position.
    This class stores the following errors:
    - actuator: Not None when the position of the actuator is out of either the
      upper or the lower bound. The actuator must shift in the opposite
      direction to get to a valid position. The sign of the value indicates if
      it is out of the upper or the lower bound.
    - wheel: Not None when the wheel is inside the stair, meaning that the
      actuator must be lifted to get the wheel out of the stair.
    - horizontal: Not None when the wheel is inside the stair, meaning that
      the structure must move horizontally in the opposite direction to get to
      a valid position.
    - incline: if a wheel collides with the stair, or is set in an unstable
      position, this value indicates the height the structure must incline to
      move the wheel horizontally to place it back to a valid position.
      Remember that all the wheels but the rear one moves horizontally when
      inclining the structure.

    """

    def __init__(self, vertical=None, wheel=None, horizontal=None):
        if horizontal is None and vertical is None and wheel is None:
            # Only when all errors are None is when the actuator is in a valid
            # position. In this case, only save the "super" correct value, and
            # does not store any other value.
            super().__init__()
            return
        # In case of error, save all the measure errors. Note that some of
        # these values can be None.
        super().__init__(False)
        self.horizontal = horizontal
        self.vertical = vertical
        self.wheel = wheel


class InclineActuatorError(ActuatorError):
    """
    When the structure collides with an actuator when elevating, one solution
    can be inclining the structure to make more room for the colliding actuator
    to complete the motion. This class stores the inclination that must be done
    to acomplish this. Since the inclination can be done fixing any of the
    four actuator, this class stores the inclination needed for each actuator
    (actually only three since the colliding actuator can not be used for
    this).
    Also, when the structure inclines, all the wheels but the rear one moves
    horizontally. This motion can provoque a wheel collision, or a pair
    unstability. This class also stores the inclination that must be done to
    the structure to take the wheel back to a valid position.

    """

    def __init__(self, actuator, incline, advance):
        """
        Arguments:
        - actuator (see ActuatorError).
        - incline -- height that the structure must incline from a given
            actuator to make room for the colliding actuator to shift
            correctly without collision. It is an array with four values, one
            for the inclination if fixing the corresponding actuator when
            inclining.
        - advance: if a wheel collides with the stair, or is set in an
                unstable position, this value indicates the height the
                structure must incline to move the wheel horizontally to place
                it back to a valid position. Remember that all the wheels but
                the rear one moves horizontally when inclining the structure.

        """
        # In this case, if actuator is correct, the other two values MUST be
        # correct also.
        if actuator:
            super().__init()
        # Otherwise, save all the values of the internat actuator.
        super().__init__(
            actuator.vertical, actuator.wheel, actuator.horizontal)
        self.incline = incline
        self.advance = advance


class HorVerError(ErrorDistance):
    """
    Class to organize horizontal - vertical error distances. This class is
    used to set the distance to stable errors (see wheel.distance_to_stable
    function).
    """

    def __init__(self, horizontal=None, vertical=None):
        if horizontal is None and vertical is None:
            super().__init__()
            return
        super().__init__(False)
        self.horizontal = horizontal
        self.vertical = vertical


class PairError(ErrorDistance):
    """
    This class evaluate whether the pair of wheels is in a stable position.
    That means that at any time at least one of the wheels must be in contact
    with the ground.

    """

    def __init__(self, rear, front, incline_rear=None, incline_front=None):
        # Rear and front are the hor-ver values needed to place the wheel in
        # a stable position. For a pair to be stable, it is enough that only
        # one of them be stable.
        # A pair is stable if one of the hor-ver objects is True.
        if rear or front:
            super().__init__()
            return
        # The pair is not stable, so we need to save all the information needed
        # by the control module to set the pair stable again.
        super().__init__(False)
        # Compute the horizontal distance from both distances.
        if rear.horizontal is None and front.horizontal is None:
            # Both wheels are over the stair, so no horizontal distance is
            # needed.
            self.horizontal = None
            self.vertical = None
        elif rear.horizontal is None:
            # Rear wheel is over the stair, so the horizontal distance tells
            # the distance to place the unstable wheel to a stable position.
            self.horizontal = front.horizontal
            self.vertical = front.vertical
            self.index = 1
        elif front.horizontal is None:
            # The opposite as above.
            self.horizontal = rear.horizontal
            self.vertical = rear.vertical
            self.index = 0
        else:
            # Both wheels are unstable, which is rare, but if happens, set
            # the horizontal distance to the minimum of both.
            if front.horizontal < rear.horizontal:
                self.horizontal = front.horizontal
                self.vertical = front.vertical
                self.index = 1
            else:
                self.horizontal = rear.horizontal
                self.vertical = rear.vertical
                self.index = 0
        # Finally, actuator stores the values each actuator need to shift to
        # place the wheel back to the stair. It is the raw information given
        # above.
        self.actuator = (rear.vertical, front.vertical)
        self.incline = (incline_rear, incline_front)


class InclinationError(ErrorDistance):
    """
    This class checks whether the structure has reach its maximum inclination.
    """

    def __init__(self, inclination):
        if inclination is None:
            super().__init__(True)
            return
        super().__init__(False)
        self.inclination = inclination

###############################################################################
###############################################################################
###############################################################################


class StructureError():
    """
    Main class. This class stores the error information for all the elements
    of the structure, check if the structure is in a valid position, and
    computes the motion required to set the structure back to a valid position.

    """

    def __init__(self, actuators, pairs, incline):
        self.actuators = actuators
        self.pairs = pairs
        self.incline = incline

    # The class only return True if there is no error in any element of the
    # structure.
    def __bool__(self):
        if not all(self.actuators):
            return False
        if not all(self.pairs):
            return False
        if not self.incline:
            return False
        return True

    def __str__(self):
        if bool(self):
            return "OK"
        return "Error"

    def horizontal(self):
        """
        This function returns the horizontal motion that the structure has to
        perform to place both pairs of wheels in a stable position.
        """
        # Check for possible collision of any wheel with the stair.
        pos_distance = 0.0
        neg_distance = 0.0
        # In case there is a number of collisions, we have to take the greater
        # (in absolute value) of all of them.
        for a in self.actuators:
            if not a:
                if a.horizontal > pos_distance:
                    pos_distance = a.horizontal
                if a.horizontal < neg_distance:
                    neg_distance = a.horizontal
        # Do the same for pair unstability.
        for p in self.pairs:
            if not p:
                if p.horizontal > pos_distance:
                    pos_distance = p.horizontal
                if p.horizontal < neg_distance:
                    neg_distance = p.horizontal

        # Check if the motion if positive (move forwards) or negative (move
        # backwards).
        if pos_distance == 0.0:
            if neg_distance == 0.0:
                # There is no horizontal motion. This should not happens, but
                # is place here to prevent runtime errors.
                return 0.0
            else:
                # Backwards motion.
                return neg_distance
        else:
            if neg_distance == 0.0:
                # Forwards motion.
                return pos_distance
            else:
                # We have both positive and negative motion. This is rare, so
                # place a mark here just in case it happens.
                raise RuntimeError

    def elevation(self):
        """
        This function returns the distance the structure has to elevate to
        place the structure in a valid position. This function must be called
        when there is a collision with any of the actuators.

        """
        pos_height = 0.0
        neg_height = 0.0

        # Get the greatest distance from all the actuators.
        for a in self.actuators:
            if not a:
                if a.vertical > pos_height:
                    pos_height = a.vertical
                if a.vertical < neg_height:
                    neg_height = a.vertical

        return pos_height \
            if abs(pos_height) > abs(neg_height) \
            else neg_height
        # # If there are two errors in opposite directions, this is caused by
        # # a internal actuator colliding with an actuator at each side. In this
        # # return 0, since no one value is better than the other
        # # TODO: Check if we can do it better.
        # if pos_height > 0 and neg_height < 0:
        #     return 0.0
        #
        # if pos_height > 0:
        #     return pos_height
        # if neg_height < 0:
        #     return neg_height
        # return 0.0

    def shift_actuator(self, index):
        """
        This function returns the shift for an actuator to place it to a valid
        position.

        """
        # Get the actuator from the actuators array.
        actuator = self.actuators[index]
        # Check if the actuator (or the wheel) is in a non-valid position.
        if not actuator:
            # In this case, the actuator is in a non valid position.
            if actuator.wheel < 0:
                # In this case, the wheel is in a non valid position (the
                # actuator can be in a valid position or not) and so,
                # return the greater of both distances. Note that the distances
                # are measured in negative values, for that reason, we have
                # to return the minimum of both (the greater in absolute
                # value).
                return min((actuator.wheel, actuator.vertical))
            # If the wheel is in a valid position, return the error for the
            # actuator.
            return actuator.vertical

        # If we reach this code, means that the actuator is in a correct
        # position, and the problem can be the pair stability.
        # Get the pair index, and the actuator index inside the pair from the
        # actuator index.
        pair_index = index // 2
        act_index = index % 2
        if not self.pairs[pair_index]:
            # Return the distance the actuator need to be shifted to place the
            # wheel back in the stair.
            return self.pairs[pair_index].actuator[act_index]
        raise RuntimeError

    def maximum_inclination(self):
        if not self.incline:
            return self.incline.inclination
        return None

    def colliding_actuator(self, fixed):
        """Find the actuator that is colliding with the structure.

        Return the index of the actuator that is the one that have collided
        the most with the structure. Return None if there is no collision with
        any actuator.

        """
        # The actuator can be colliding from the upper or the lower bound. So,
        # we have to record which actuator is colliding, and from which bound
        # the actuator has colllided.
        max_index = None
        max_value = 0.0
        min_index = None
        min_value = 0.0

        for n in range(4):
            if not self.actuators[n]:
                error = self.actuators[n].incline[fixed]
                if error > max_value:
                    max_value = error
                    max_index = n
                if error < min_value:
                    min_value = error
                    min_index = n

        if max_index is not None and min_index is not None:
            raise RuntimeError
        if max_index is not None:
            return max_index
        if min_index is not None:
            return min_index
        return None

    def inclination(self, fixed=0):
        """
        This function returns the height the structure has to incline to
        place the structure in a valid position.

        """
        # In clase there are more than one collision, we have to choose the
        # greater of them. We use these twro variables to get the one with the
        # maximum absolute value.
        pos_incline = 0.0
        neg_incline = 0.0

        if not self.incline:
            if self.incline.inclination > pos_incline:
                pos_incline = self.incline.inclination
            if self.incline.inclination < neg_incline:
                neg_incline = self.incline.inclination

        # Check if there is a collision with any of the actuators. To
        # understand the value stored in actuators array, see function
        # joint.inverse_prop_lift().
        for n in range(4):
            if n == fixed:
                continue
            if not self.actuators[n]:
                error = self.actuators[n].incline[fixed]
                if error > pos_incline:
                    pos_incline = error
                if error < neg_incline:
                    neg_incline = error

        # Apart from the inclination, we also have to check whether any wheel
        # has collided with the stair.
        for actuator in self.actuators[1:]:
            # Only need to check actuators 1 to 3. The first one not need to
            # be checked, since this actuator does not move when inclining.
            if not actuator:
                if actuator.advance > pos_incline:
                    pos_incline = actuator.advance
                if actuator.advance < neg_incline:
                    neg_incline = actuator.advance

        # Also check if after the inclination any pair has been placed in an
        # unstable position.
        for pair in self.pairs:
            if not pair:
                stb_incline = pair.incline[pair.index]
                if stb_incline > pos_incline:
                    pos_incline = stb_incline
                if stb_incline < neg_incline:
                    neg_incline = stb_incline

        return pos_incline \
            if abs(pos_incline) > abs(neg_incline) \
            else neg_incline
        # for pair in self.pairs:
        #     if pair.incline[]
        # And also check if after the motion due to the inclination, any pair
        # of wheels are in a unstable position.
        # if not self.pairs[0]:
        #     if self.pairs[0].incline[self.pairs[0].index] > pos_incline:
        #         pos_incline = self.pairs[0].incline[self.pairs[0].index]
        #     if self.pairs[0].incline[self.pairs[0].index] < neg_incline:
        #         neg_incline = self.pairs[0].incline[self.pairs[0].index]
        # if not self.pairs[1]:
        #     if self.pairs[1].incline[self.pairs[1].index] > pos_incline:
        #         pos_incline = self.pairs[1].incline[self.pairs[1].index]
        #     if self.pairs[1].incline[self.pairs[1].index] < neg_incline:
        #         neg_incline = self.pairs[1].incline[self.pairs[1].index]

        # # If there are two errors in opposite directions, this is caused by
        # # a internal actuator colliding with an actuator at each side. In this
        # # return 0, since no one value is better than the other
        # # TODO: Check if we can do it better.
        # if pos_incline > 0 and neg_incline < 0:
        #     return 0.0
        #
        # if pos_incline > 0:
        #     return pos_incline
        # if neg_incline < 0:
        #     return neg_incline
        # return 0.0

    def push_actuator(self, index):
        """
        This function return the height the structure must be pushed to allow
        an actuator to complete a motion.

        NOTE: This function is not the same than shift_actuator. This is
        intended to compute the height the structure must be pushed, while the
        other function is just to compute the height the actuator must be
        shift to place it in a valid position.

        """
        # Get the actuator from the actuators array.
        actuator = self.actuators[index]
        # Check if the actuator is in a non-valid position.
        if not actuator:
            if actuator.vertical > 0:
                # If the error is positive, means that the actuator has gone
                # out from the upper bound. In this case, just return this
                # value.
                return actuator.vertical
            # However, if the value is negative, the error can be due to the
            # actuator going out from the lower bound, or the wheel colliding
            # with the ground.
            height = actuator.vertical - actuator.wheel
            return height if height < 0 else 0.0
        return 0.0
