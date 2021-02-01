'''
Created on 29 ene. 2021

@author: pedro.gil@uah.es

Module to define a pair of wheels. This pair of wheels holds some properties
for the rear and the front pair.
'''

class ActuatorPair:
    
    def __init__(self, rear, front):
        """Constructor:
        
        Save the rear and front actuators.
        
        """
        self.REAR = rear
        self.FRONT = front
        
    def check_collision(self, distance):
        """Check if any of the wheels (or both) are in a forbidden position.
        
        Returns:
        - True if any of the wheels have collided, False otherwise.
        - If True, returns the distance the pair need to be moved to place
            it in a safe position.
        
        """
        # Check for possible wheel collisions.
        fr_res, fr_dis, __ = self.REAR.move_actuator()
        re_res, re_dis, __ = self.FRONT.move_actuator()
        if not fr_res:
            # If the front wheel have collided,
            if not re_res:
                # Both wheels have collided. Get the maximum distance.
                if distance > 0:
                    dis = min([fr_dis, re_dis])
                else:
                    dis = max([fr_dis, re_dis])
            else:
                # In this case, only the front wheel have collided.
                dis = fr_dis
            res = False
        elif not re_res:
            # In this case, only the rear wheel has collided.
            dis = re_dis
            res = False
        else:
            # In this case, none of the wheels have collided.
            res = True
            dis = 0.0
        return res, dis
            
    def check_stable(self, distance):
        """Move the pair of actuators, because the structure has move.
        
        This function check if oth wheels get in unstable position (at any
        time, at least one wheel must remain stable).
            
        Parameters:
        - Distance: distance to move. It is only its sign what is important for
            the function, not its value.
            
        Returns:
        - True if the motion succeed. False otherwise.
        - If False, returns the distance the pair need to be moved to place
            it in a safe position. The sign of this distance is always the
            opposite of the distance given.
        
        """
        # Check if either wheel is in a stable position.
        fr_grd = self.REAR.ground()
        re_grd = self.FRONT.ground()
        
        if not fr_grd and not re_grd:
            # Both wheels are unstable. Get the minimum distance the pair has
            # to be moved to place one of the wheels back to a stable position.
            re_grd_dis = self.REAR.back_to_stable()
            fr_grd_dis = self.FRONT.back_to_stable()
            # Get the minimum value of both distances. In this case, we need
            # the minimum, since this is the distance to get the structure back
            # to a safe position.
            if distance > 0:
                dis = max([fr_grd_dis, re_grd_dis])
            else:
                dis = min([fr_grd_dis, re_grd_dis])
            res = False
        else:
            # Al least one wheel is stable, so that the structure in safe.
            res = True
            dis = 0

        return res, dis

    def shift_actuator(self, actuator, distance, check):
        """Shift the given actuator.
        
        Returns:
        - False if an error has happened. These errors can be:
            - The actuator has reached one of its ends.
            - The wheel has collided with the stair.
            
        Parameters:
        - actuator:
            - 0: Rear actuator.
            - 1: Front actuator.
        - distance: distance to move the actuator. Positive value means the
            the wheel moves away from the structure.
        - check: At any time, at least one wheel must be in a stable position.
            If this value is False, this check is not performed.
        
        """
        if actuator == 0:
            # Rear actuator.
            # Shift an actuator is only possible when the other wheel is on the
            # ground. Check if this is correct.
            if check and not self.FRONT.ground():
                return False, 0
            res, dis = self.REAR.shift_actuator(distance)
        elif actuator == 1:
            # Front actuator.
            if check and not self.REAR.ground():
                return False, 0
            res, dis = self.FRONT.shift_actuator(distance)
        else:
            raise RuntimeError("Error in shft_actuator")
        
        return res, dis
        

    # =========================================================================
    # Drawing functions.
    # =========================================================================
    def position(self, height):
        xr, yr = self.REAR.JOINT.position(height)
        xf, yf = self.FRONT.JOINT.position(height)
        return xr, yr, xf, yf
      
    def draw(self, origin, image, scale, shift):
        self.FRONT.draw(origin, image, scale, shift)
        self.REAR.draw(origin, image, scale, shift)
        
###############################################################################
# End of file.
###############################################################################
