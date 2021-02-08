'''
Created on 29 ene. 2021

@author: pedro.gil@uah.es

The structure can be considered as a set of two sets of a pairs of wheels.
This module define the functionality of the two pairs of wheels.

'''

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
       
    def check_collision(self, distance):
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
                if distance > 0:
                    hor = min([fr_hor, re_hor])
                    ver = min([fr_ver, re_ver])
                    act = min([fr_act, re_act])
                else:
                    hor = max([fr_hor, re_hor])
                    ver = max([fr_ver, re_ver])
                    act = max([fr_act, re_act])
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

    def check_stable(self, distance):
        """Check the position of the pair of wheels.
        
        This function check if the wheels get in unstable position (at any
        time, at least one wheel must remain stable).
            
        Parameters:
        - Distance: distance that the wheels have moved prior the current
            check.
            
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
                    if distance > 0:
                        dis = max([fr_grd_dis, re_grd_dis])
                    else:
                        dis = min([fr_grd_dis, re_grd_dis])
            return False, dis
        else:
            # Al least one wheel is stable, so that the structure in safe.
            return True, 0.0
  
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
