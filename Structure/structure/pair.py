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
        """Check if any both wheels have collided with the stair.
        
        
        
        """
        # Check for possible wheel collisions.
        fr_res, fr_dis, __ = self.REAR.move_actuator()
        re_res, re_dis, __ = self.FRONT.move_actuator()
        if not fr_res:
            if not re_res:
                # Both wheels have collided. Get the maximum distance.
                if distance > 0:
                    dis = min([fr_dis, re_dis])
                else:
                    dis = max([fr_dis, re_dis])
            else:
                dis = fr_dis
            res = False
        elif not re_res:
            dis = re_dis
            res = False
        else:
            res = True
            dis = 0.0
        return res, dis
            
    def check_stable(self, distance):
        """Move the pair of actuators, because the structure has move.
        
        This function check for any error when moving the structure. These
        errors are:
        - Collision of any wheel with a step of the stair.
        . Both wheels get in unstable position (at any time, at least one
            wheel must remain stable.
            
        Parameters:
        - Distance: distance to move. It is only its sign what is important for
            the function, not its value.
            
        Returns:
        - True if the motion succeed. False otherwise.
        - I False, return also the maximum allowed distance the structure can
            move safely. The sign of this distance is always the opposite of
            the distance given.
        
        """
        # Check if either wheel is in a stable position.
        fr_grd = self.REAR.ground()
        re_grd = self.FRONT.ground()
        
        if not fr_grd and not re_grd:
            # Both wheels are unstable. Get the minimum distance the pair has
            # to be moved to place one of the wheels back to a stable position.
            re_grd_dis = self.REAR.back_to_stable()
            fr_grd_dis = self.FRONT.back_to_stable()
            # Get the minimum value of both distances.
            if distance > 0:
                dis = max([fr_grd_dis, re_grd_dis])
            else:
                dis = min([fr_grd_dis, re_grd_dis])
            res = False
        else:
            res = True
            dis = 0

        return res, dis

    def shift_actuator(self, actuator, distance, check):
        if actuator == 0:
            # Shift an actuator is only posible when the other wheel is on the
            # ground. Check if this is correct.
            if check and not self.FRONT.ground():
                return False, 0
            res, dis = self.REAR.shift_actuator(distance)
        elif actuator == 1:
            if check and not self.REAR.ground():
                return False, 0
            res, dis = self.FRONT.shift_actuator(distance)
        else:
            raise RuntimeError("Error in shft_actuator")
        
        return res, dis
        
    def position(self, height):
        xr, yr = self.REAR.JOINT.position(height)
        xf, yf = self.FRONT.JOINT.position(height)
        return xr, yr, xf, yf
    # =========================================================================
    # Drawing functions.
    # =========================================================================
   
    def draw(self, origin, image, scale, shift):
        self.FRONT.draw(origin, image, scale, shift)
        self.REAR.draw(origin, image, scale, shift)