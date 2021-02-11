'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Part of the structure that fix the actuator to the main base. This module
implements the trigonometric computation to calculate the position of an
actuator according to the main base inclination. 

'''

from math import cos, sin, sqrt

# =============================================================================
# Joint definition:
# =============================================================================

class Joint:
    """Class to define the joint of an actuator with the top of the structure.
    """
    def __init__(self, base, x):
        """Constructor:
        
        Parameters:
        base -- Reference to the actual base that hold the joint. Used to 
          compute absolute coordinates from the offset of the joint with respect
          to the base origin.
        x -- Horizontal distance of the actuator to the back side of the
        structure.
        
        """
        self.base = base
        self.x = x

    def position(self, height=0):
        """Returns the (x, y) position of a given point along the actuator.
        
        Parameters:
        height -- Vertical distance from the required point to the joint.
        
        """
        # Copy global variables of the structure.
        shift = self.base.shift
        elevation = self.base.elevation
        angle = self.base.angle
        # Get actual coordinates.
        x = shift + self.x*cos(angle)
        y = elevation - height + self.x*sin(angle)

        return x, y

    def proportional_lift(self, height):
        """Computes a lift when inclining the structure.
        
        Computes the shift for an inner actuator when the outer actuator is
        shift the given distance when inclining the structure.
        
        Returns the required distance.
        
        """
        return height*self.x/self.base.WIDTH

    def inverse_prop_lift(self, height):
        """Function inverse to function proportional_lift.
        
        The function returns the shift the exterior actuators need to move to
        get the current actuator shift the given height when inclining. The
        values returned are:
        - Height for the front actuator.
        - Height for the rear actuator.
        
        This function can only be called for the interior actuators. For the
        exterior actuators this value will be 0 and infinity.
        
        """
        if height == 0.0 or self.x == 0 or self.base.WIDTH == self.x:
            return {
                "front": 0.0,
                "rear": 0.0}
        fr_height = height*self.base.WIDTH / self.x
        re_height = height*self.base.WIDTH / (self.base.WIDTH-self.x)
        return { 
            "front": fr_height,
            "rear": re_height}
        

    def lift_from_horizontal_motion(self, distance, front):
        """Computes a vertical motion when inclining the structure.
        
        Computes a vertical motion of an actuator to achieve a horizontal
        motion when the structure is inclined
        (see liftfrom_horizontal_motion.svg).
        
        Returns the horizontal shift for the actuator.
        
        Parameters:
        distance -- Horizontal distance to achieve.
        front -- see function incline in structure.base.
        
        """
        # Obtain actual coordinates for the current joint.
        xa, ya = self.position()
        x0 = xa-self.base.shift
        y0 = ya-self.base.elevation
        if front:
            # If the fixed wheel is the front one, we need to update the
            # distance to move, taking into account that, when inclining
            # the structure, both, the front and the actual, wheels moves.
            # So, the distance must be the difference between both:
            # d_a=distance
            # d_f=distance*W/x
            # Reorganizing...
            # d=d_a-d_f=d_a*x/(x-W)
            distance *= self.x/(self.x-self.base.WIDTH)
        # Compute target horizontal coordinate.
        x1 = x0+distance
        # The value for y can be computed from the Pithagoras theorem.
        # Nevertheless, we need to choose the sign of the square root.
        # Since the motion required is small, the new height
        # must match the sign of the old one.
        if y0 > 0:
            y1 = +sqrt(self.x**2 - x1**2)
        else:
            y1 = -sqrt(self.x**2 - x1**2)
        return y1-y0
    
###############################################################################
# End of file.
###############################################################################
    