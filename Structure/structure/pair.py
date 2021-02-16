'''
Created on 29 ene. 2021

@author: pedro.gil@uah.es

The structure can be considered as a set of two sets of a pairs of wheels.
This module define the functionality of the two pairs of wheels.

'''

from math import isinf, inf

from control.distance_errors import merge_collision, merge_stability, StabilityErrors

EDGE_MARGIN = 4.0


class ActuatorPair:
    
    def __init__(self, rear, front, rear_pair):
        """Constructor:
        
        Parameters:
        - rear: Rear actuator.
        - front: Front actuator.
        - rear_pair: If true, this is the pair placed in the rear part of the
            structure.
        
        """
        self.REAR = rear
        self.FRNT = front
        self.REAR_PAIR = rear_pair
        
    def shift_actuator(self, rear, front, distance):
        """Shift the given actuator.
            
        Parameters:
        - rear: If True, shift this actuator.
        - front: If True, shift this actuator.
        - distance: distance to move the actuator. Positive value means the
            the wheel moves away from the structure.
        
        """
        if rear:
            # Rear actuator.
            self.REAR.shift_actuator(distance)
        if front:
            # Front actuator.
            self.FRNT.shift_actuator(distance)
        
    def shift_actuator_proportional(self, rear, front, distance):
        """Similar to shift_actuator, but proportional.
        
        """
        if rear:
            self.REAR.shift_actuator_proportional(distance)
        if front:
            self.FRNT.shift_actuator_proportional(distance)
       
    def check_collision(self):
        """Check if any of the wheels (or both) are in a forbidden position.
        
        Returns:
        - False if any of the wheels have collided, True otherwise.
        - If True, returns the distance the pair need to be moved to place
            it in a safe position, both in horizontal and in vertical, and for
            the actuator.
        
        """
        # TODO: Add comments for inverse_lift
        # Check for possible wheel collisions.
        fr_res = self.FRNT.check_actuator()
        re_res = self.REAR.check_actuator()

        res = merge_collision(fr_res, re_res)
        
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
    
#         if not fr_res:
#             # If the front wheel have collided,
#             if not re_res:
#                 # Both wheels have collided. Get the largest distance.
#                 # NOTE: Take into account that the error can be negative or
#                 # positive, but always both are either positive or negative.
#                 # For that reason, we compare the absolute value of both
#                 # distances, and choose the largest one, keeping its actual
#                 # sign.
#                 if abs(fr_hor) > abs(re_hor):
#                     hor = fr_hor
#                 else:
#                     hor = re_hor
#                 if abs(fr_ver) > abs(re_ver):
#                     ver = fr_ver
#                 else:
#                     ver = re_ver
#                 if abs(fr_act) > abs(re_act):
#                     act = fr_act
#                 else:
#                     act = re_act
#             else:
#                 # In this case, only the front wheel have collided.
#                 hor = fr_hor
#                 ver = fr_ver
#                 act = fr_act
#         elif not re_res:
#             # In this case, only the rear wheel has collided.
#             hor = re_hor
#             ver = re_ver
#             act = re_act
#         else:
#             # In this case, none of the wheels have collided.
#             errors = CollisionErrors(True)
#             return errors
#         errors = CollisionErrors(False, hor, ver, act, re_inc, fr_inc)
#         return errors
    
    def check_stable(self):
        """Check the position of the pair of wheels.
        
        This function check if the wheels get in unstable position (at any
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
            # Both wheels are not on the ground:
            # Get the minimum distance the pair has to be moved to place one of
            # the wheels back to a stable position.
            fr_stb = self.FRNT.distance_to_stable()
            re_stb = self.REAR.distance_to_stable()
            
            res = StabilityErrors(False, fr_stb, re_stb)
            return res
        return StabilityErrors()
    
    
#             if re_grd_dis is None:
#                 if fr_grd_dis is None:
#                     # This happens when shifting one actuator with the other
#                     # already in the air.
#                     dis = 0.0
#                 else:
#                     dis = fr_grd_dis
#             else:
#                 if fr_grd_dis is None:
#                     dis = re_grd_dis
#                 else:
#                     # Get the minimum value of both distances. In this case, we
#                     # need the minimum, since this is the distance to get the
#                     # structure back to a safe position.
#                     if abs(fr_grd_dis) < abs(re_grd_dis):
#                         dis = fr_grd_dis
#                     else:
#                         dis = re_grd_dis
#             error = StabilityErrors(False, dis)
#             return error
#         # At least one wheel is stable, so that the structure in safe.
#         error = StabilityErrors(True)
#         return error
    
    # =========================================================================
    # Control functions.
    # =========================================================================
    
    def get_wheel_distances(self):
        """Get distances from wheels to stairs for control module.
        
        """
        # Get distances for both wheel of the pair.
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        
        if isinf(fr_res['wr']):
            # The front wheel has reached the end of the stair.
            if isinf(re_res['wr']):
                # The rear wheel has also reached the end of the stair.
                return 0, inf, min([fr_res['hr'], re_res['hr']])
            else:
                fr_res['up'] = re_res['up']
                try:
                    fr_res['wr'] = fr_res['wc']
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
            # Check for possible unstabilities when shifting the wheel.
            if not passive['st']:
                # The passive wheel is not on the ground, so that we can not
                # elevate the active wheel before the passive is on the ground.
                # Divide the instruction into two. The first one will be used
                # to take the passive wheel to the ground, and the second to
                # complete the motion for the active wheel, but this will be
                # done, hopefully, in the following iteration.
                # NOTE: Although the key hc is not always present, for this
                # case, that is, the chosen wheel is the one that is on the
                # ground, it is not possible that the passive wheel is in an
                # unstable position. If this happened, it will be impossible
                # to pass the stair.
                # Compute the total height to shift between both actuators.
                ver_total = abs(active['hr']) + abs(passive['hc'])
                # Divide the horizontal distance to advance proportional to
                # both heights.
                k = abs(passive['hc']) / ver_total
                hor = k * active['wr']
                # Now, take the passive actuator to the ground.
                ver = passive['hc']
                # Change the wheel to move.
                index = (index + 1) % 2
                # The passive wheel is on the ground, so that we need not take
                # care of this wheel.
            if ver > 0:
                ver += EDGE_MARGIN
            else:
                hor += EDGE_MARGIN            
        elif not re_res['up'] and not fr_res['up']:
            if passive['st']:
                # If the passive wheel is on the ground, check if the active
                # wheel can be moved pass the edge of the step.
                try:
                    hor = active['wc'] + EDGE_MARGIN
                except KeyError:
                    pass
            else:
                # Else, if the passive wheel is not on the ground, but the
                # active wheel does, use this instruction to take the passive
                # wheel to the ground as the active wheel moves the distance
                # required (to be still stable).
                # Change the active wheel
                index = (index + 1) % 2
                # Note that if the active wheel is stable, the vertical
                # distance must be 0 (you can not take a wheel down if it is
                # on the ground).
                ver = passive['hr']
        else:
            raise NotImplementedError("Wheel facing steps with different sign.")
        
        return index, hor, ver
        
    def set_to_ground(self):
        """Return the distance to place both wheel on the ground, if possible.
        
        """
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        if not re_res['st'] and fr_res['st']:
            return 0, re_res['hr']
        elif not fr_res['st'] and re_res['st']:
            return 1, fr_res['hr']
        elif fr_res['st'] and re_res['st']:
            return None
        else:
            raise RuntimeError("Both wheel on the air.")

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
        
###############################################################################
# End of file.
###############################################################################
