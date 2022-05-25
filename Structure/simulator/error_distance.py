'''
Created on 20 may. 2022

@author: pedrogil

Definition of classes to return the list of distances errors when checking the
position of the structure with respect to the stairs.
'''


class ErrorDistance():
    """
    Generic class. All classes derive from this one.
    """

    # This class save the state of the measures. If every thing is OK, this
    # object set correct equal True, and does not store any other data.
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
      the structure must move in the opposite direction to get to a valid
      position.

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

    # def set_inclination(self, height):
    #     self.inclination = height
    #
    # def get_inclination(self):
    #     return self.inclination
    #
    # inclination = property(get_inclination, set_inclination, None, None)
    # TODO: Comment this.
    # def get_front(self):
    #     return self.horizontal
    #
    # def get_rear(self):
    #     return self.horizontal
    #
    # front = property(get_front, None, None, None)
    # rear = property(get_rear, None, None, None)


class FrontActuatorError(ActuatorError):
    """
    For the front actuator, apart from collisions for the actuator, it also
    can have errors for collisions when incline. This class includes this value

    """

    def __init__(self, actuator, incline=None):
        # In this case, if actuator is correct, the other two values MUST be
        # correct also.
        if actuator:
            super().__init()
        # Otherwise, save all the values of the internat actuator.
        super().__init__(
            actuator.vertical, actuator.wheel, actuator.horizontal)
        self.incline = incline


class InternalActuatorError(ActuatorError):
    """
    For internal actuators, when the base collides with one of these two
    actuators, this class combines all this information:
    - Normal actuators (see ActuatorError).
    - For internal actuators, if the structure base collides with one of the
      internal actuators, these values means (see inv_proportional_shift.svg):
      - front: distance to elevate from the front so that the rear end of the
            base can get to its desired position.
      - rear: similar to front, but the other way around.
      - incline: if a wheel collides with the stair, or is set in an unstable
            position, this value indicates the height the structure must
            incline to take the wheel back to a valid position.
    """

    def __init__(self, actuator, rear=None, front=None, incline=None):
        # In this case, if actuator is correct, the other two values MUST be
        # correct also.
        if actuator:
            super().__init()
        # Otherwise, save all the values of the internat actuator.
        super().__init__(
            actuator.vertical, actuator.wheel, actuator.horizontal)
        self.rear = rear
        self.front = front
        self.incline = incline


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
    This class evaluate if the pair of wheels is in a stable position. That
    means that at any time at least one of the wheels must be in contact with
    the ground.

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
            # raise RuntimeError
        # Finally, actuator stores the values each actuator need to shift to
        # place the wheel back to the stair.
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

        if pos_height == 0.0:
            if neg_height == 0.0:
                # There is no collision with any actuator (this must not
                # happens, but leave here to prevent errors).
                return 0.0
            else:
                # The structure must be taken down.
                return neg_height
        else:
            if neg_height == 0.0:
                # The structure must be elevated.
                return pos_height
            else:
                # Some actuators are collided from the upper bound, and other
                # from the lower. This must not happens.
                raise RuntimeError

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

    def inclination(self, rear=False):
        """
        This function returns the height the structure has to incline to
        place the structure in a valid position.
        """
        pos_incline = 0.0
        neg_incline = 0.0
        if not self.incline:
            # Check if the structure has reached the maximum inclination
            if self.incline.inclination > 0:
                pos_incline = self.incline.inclination
            else:
                neg_incline = self.incline.inclination

        # But also we have to check if there is a collision with any of the
        # actuators.
        if not rear:
            # If we elevate from the front, the collision can happens with all
            # the actuators but the rear one.
            if not self.actuators[3]:
                # For the frontal actuator, we only have to check the distance
                # error of this actuator.
                if self.actuators[3].vertical > pos_incline:
                    pos_incline = self.actuators[3].vertical
                if self.actuators[3].vertical < neg_incline:
                    neg_incline = self.actuators[3].vertical
            if not self.actuators[2]:
                # But for the actuator 2, we have to check the frontal value.
                if self.actuators[2].front > pos_incline:
                    pos_incline = self.actuators[2].front
                if self.actuators[2].front < neg_incline:
                    neg_incline = self.actuators[2].front
            if not self.actuators[1]:
                # And the same for the actuator 1.
                if self.actuators[1].front > pos_incline:
                    pos_incline = self.actuators[1].front
                if self.actuators[1].front < neg_incline:
                    neg_incline = self.actuators[1].front

        else:
            # Otherwise, if we elevate from the rear, the collision can happen
            # with all the actuator but the front one (see comments above).
            # Note that we have to change signs from the actuator values.
            if not self.actuators[0]:
                if -self.actuators[0].vertical > pos_incline:
                    pos_incline = -self.actuators[0].vertical
                if -self.actuators[0].vertical < neg_incline:
                    neg_incline = -self.actuators[0].vertical
            if not self.actuators[1]:
                if -self.actuators[1].rear > pos_incline:
                    pos_incline = -self.actuators[1].rear
                if -self.actuators[1].rear < neg_incline:
                    neg_incline = -self.actuators[1].rear
            if not self.actuators[2]:
                if -self.actuators[2].rear > pos_incline:
                    pos_incline = -self.actuators[2].rear
                if -self.actuators[2].rear < neg_incline:
                    neg_incline = -self.actuators[2].rear

        # Apart from the inclination, we also have to check whether any wheel
        # has collided with the stair.
        if not self.actuators[3]:
            # For the frontal actuator, we only have to check the distance
            # error of this actuator.
            if self.actuators[3].incline > pos_incline:
                pos_incline = self.actuators[3].incline
            if self.actuators[3].incline < neg_incline:
                neg_incline = self.actuators[3].incline
        if not self.actuators[2]:
            # But for the actuator 2, we have to check the frontal value.
            if self.actuators[2].incline > pos_incline:
                pos_incline = self.actuators[2].incline
            if self.actuators[2].incline < neg_incline:
                neg_incline = self.actuators[2].incline
        if not self.actuators[1]:
            # And the same for the actuator 1.
            if self.actuators[1].incline > pos_incline:
                pos_incline = self.actuators[1].incline
            if self.actuators[1].incline < neg_incline:
                neg_incline = self.actuators[1].incline

        if not self.pairs[0]:
            if self.pairs[0].incline[self.pairs[0].index] > pos_incline:
                pos_incline = self.pairs[0].incline[self.pairs[0].index]
            if self.pairs[0].incline[self.pairs[0].index] < neg_incline:
                neg_incline = self.pairs[0].incline[self.pairs[0].index]
        if not self.pairs[1]:
            if self.pairs[1].incline[self.pairs[1].index] > pos_incline:
                pos_incline = self.pairs[1].incline[self.pairs[1].index]
            if self.pairs[1].incline[self.pairs[1].index] < neg_incline:
                neg_incline = self.pairs[1].incline[self.pairs[1].index]

        return pos_incline if \
            abs(pos_incline) > abs(neg_incline) else neg_incline
