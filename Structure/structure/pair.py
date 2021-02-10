'''
Created on 29 ene. 2021

@author: pedro.gil@uah.es

The structure can be considered as a set of two sets of a pairs of wheels.
This module define the functionality of the two pairs of wheels.

'''

EDGE_MARGIN = 2


class ActuatorPair:
    
    def __init__(self, rear, front):
        """Constructor:
        
        Parameters:
        - rear: Rear actuator.
        - front: Front actuator.
        
        """
        self.REAR = rear
        self.FRNT = front
        
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
        # Check for possible wheel collisions.
        fr_res, fr_hor, fr_ver, fr_act = self.REAR.check_actuator()
        re_res, re_hor, re_ver, re_act = self.FRNT.check_actuator()
        if not fr_res:
            # If the front wheel have collided,
            if not re_res:
                # Both wheels have collided. Get the largest distance.
                # NOTE: Take into account that the error can be negative or
                # positive, but always both are either positive or negative.
                # For that reason, we compare the absolute value of both
                # distances, and choose the largest one, keeping its actual
                # sign.
                if abs(fr_hor) > abs(re_hor):
                    hor = fr_hor
                else:
                    hor = re_hor
                if abs(fr_ver) > abs(re_ver):
                    ver = fr_ver
                else:
                    ver = re_ver
                if abs(fr_act) > abs(re_act):
                    act = fr_act
                else:
                    act = re_act
            else:
                # In this case, only the front wheel have collided.
                hor = fr_hor
                ver = fr_ver
                act = fr_act
            res = False
        elif not re_res:
            # In this case, only the rear wheel has collided.
            hor = re_hor
            ver = re_ver
            act = re_act
            res = False
        else:
            # In this case, none of the wheels have collided.
            return True, 0.0, 0.0, 0.0
        return res, hor, ver, act

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
            re_grd_dis = self.REAR.distance_to_stable()
            fr_grd_dis = self.FRNT.distance_to_stable()
            
            if re_grd_dis is None:
                if fr_grd_dis is None:
                    # This happens when shifting one actuator with the other
                    # already in the air.
                    dis = 0.0
                else:
                    dis = fr_grd_dis
            else:
                if fr_grd_dis is None:
                    dis = re_grd_dis
                else:
                    # Get the minimum value of both distances. In this case, we
                    # need the minimum, since this is the distance to get the
                    # structure back to a safe position.
                    if abs(fr_grd_dis) < abs(re_grd_dis):
                        dis = fr_grd_dis
                    else:
                        dis = re_grd_dis
            return False, dis
        else:
            # Al least one wheel is stable, so that the structure in safe.
            return True, 0.0
  
    # =========================================================================
    # Control functions.
    # =========================================================================
    
    def get_wheel_distances(self):
        """Get distances from wheels to stairs for control module.
        
        """
        # Get distances for both wheel of the pair.
        re_res = self.REAR.get_wheel_distances()
        fr_res = self.FRNT.get_wheel_distances()
        
        # Choose the wheel which is closest to its nearest step. This is the
        # maximum distance the wheel pair can move.
        if re_res['up'] and fr_res['up']:
            # Both wheels are facing a positive step. Select the active and
            # the passive wheel. Active wheel is the wheel which leads the motion
            # for this pair.
            if re_res['wr'] < fr_res['wr']:
                active = re_res
                passive = fr_res
                index = 0
            else:
                active = fr_res
                passive = re_res
                index = 1
            
            hor = active['wr']
            ver = active['hr']
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
                ver_total = abs(active['hr']) + abs(passive['hc'])
                k = passive['hc'] / ver_total
                hor = k * active['wr']
                ver = k * passive['hc']
                # Change the wheel to move.
                index = (index + 1) % 2
            else:  
                # The passive wheel is on the ground, so that we need not take
                # care of this wheel.
                if ver > 0:
                    ver += EDGE_MARGIN
                    hor -= EDGE_MARGIN
                else:
                    hor += EDGE_MARGIN
            
        elif re_res['up'] and fr_res['up']:
            pass
    
        else:
            raise NotImplementedError("Wheel facing steps with different sign.")
        
        return index, hor, ver
        
        
    """
def compute_motion(motion):
    Computes motion according a pair of wheels (front or rear wheels).
    
    Returns:
    -- Wheel to lift (or take down).
    -- Height for the wheel to lift (or take down).
    -- Distance for the structure to move.
    
    # Check if front wheel has reached the end of the stair.
    if motion[1] is None:
        # If so, check if also does the rear wheel.
        if motion[0] is None:
            # Both wheels have reached the end of the stair. Return an infinity
            # value to ensure this case will never be chosen.
            return 0, 0.0, None
        # The rear wheel still has not reached the end. Return the distance
        # needed to advance.
        if motion[0]['hr'] < 0:
            return 0, motion[0]['hr'], \
                motion[0]['wr'] + EDGE_MARGIN
        else:
            return 0, motion[0]['hr'], \
                motion[0]['wr'] - EDGE_MARGIN        
    # Get distances from the wheel to the next step.
    try:
        # Rear wheel:
        m_r = motion[0]['wr']
    except TypeError:
        # If an error raises on either instruction, the next instruction MUST
        # be issued to place this wheel on a stable position. Take into account 
        # that it is not possible that both wheel were in an unstable position
        # at the same time.
        return 0, motion[0]['hr'], motion[0]['wr']
    try:
        # Front wheel
        m_f = motion[1]['wr']
    except TypeError:
        return 1, motion[1]['hr'], motion[1]['wr']
    
    # In any other case, get the maximum value the structure can be moved
    # before a collision (or falling down) happens.
    if m_r < m_f:
        # The rear wheel is closer than the front one.
        active = 0
        passive = 1
    else: 
        active = 1
        passive = 0
        
    if motion[passive]['hc'] is None:
        # However, the other wheel is not on the ground, so that we can
        # not lift the active wheel.
        # So, in the first place, we need to take the other wheel down
        # before lift the current wheel.
        return passive, motion[passive]['hr'], motion[passive]['wr']
    else:
        if not motion[passive]['st']:
            # In this case, the other wheel is not on the ground, so we
            # can not lift the current wheel.
            # To reduce time, divide the total distance to move (given by
            # active wheel) in two, one part to take the passive wheel to the
            # ground, and other part to perform the current motion (but this
            # part will be performed on the next iteration).
            # Divide the distance proportional to the total height to
            # correct.
            try:
                ka = abs(motion[active]['hr'])
                kp = abs(motion[passive]['hc'])
                #TODO: Ensure that this case never happen when the key hc is
                # not included in the dictionary (unstable position).
                k = kp / (ka + kp + EDGE_MARGIN)
            except ZeroDivisionError:
                k = 1.0
            return passive, motion[passive]['hc'], k*motion[active]['wr']
        else:
            if motion[active]['hr'] < 0:
                return active, motion[active]['hr'], \
                    motion[active]['wr'] + EDGE_MARGIN
            else:
                return active, motion[active]['hr'], \
                    motion[active]['wr'] - EDGE_MARGIN      
    """
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
