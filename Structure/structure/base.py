'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to define all the elements which composes the structure:
- Actuator (x4).
- Current position with respect to the environment.
- Current elevation with respect to the ground.

'''

from math import asin
from numpy import float32
import cv2

from structure.actuator import WheelActuator
from structure.pair import ActuatorPair


class Base:
    '''
    Class to the define the whole structure.
    '''


    def __init__(self, size, wheels, stairs):
        """Constructor:
        
        Parameters:
        size -- Dictionary with the dimensions of the structure:
            a, d, c -- Partial dimensions of the base (see paper)
            d -- Height of the structure (and length of actuators).
            g -- Gap between floor and lower base.
        wheels -- Dictionary with the radius of wheels:
            r1, r2, r3, r4 -- Wheels radius (can be different).
        stairs -- Physics structure for the stairs.
        
        """
        # Main distances of the structure.
        a = size['a']
        b = size['b']
        c = size['c']
        d = size['d']
        # NOTE: The distance g has nothing to do with the control module. It is
        # just for representation purposes.
        g = size['g']
        r1 = wheels['r1']
        r2 = wheels['r2']
        r3 = wheels['r3']
        r4 = wheels['r4']
        # Current vertical position.
        self.elevation = d+g
        # Current horizontal position.
        self.shift = 0.0
        # Angle of the structure (driven by L9 actuator, see paper).
        self.angle = 0.0
        # Create array of actuators.
        # NOTE: The total length of an actuator is equal to the height of the
        # structure (d), plus the gap between the floor and the lower base of
        # the structure (g), minus the wheel radius (r).
        # TODO: Comment
        self.REAR = ActuatorPair(
                WheelActuator(0, d, d+g-r1, r1, self, stairs),
                WheelActuator(a, d, d+g-r2, r2, self, stairs))
        self.FRNT = ActuatorPair(
                WheelActuator(a+b, d, d+g-r3, r3, self, stairs),
                WheelActuator(a+b+c, d, d+g-r4, r4, self, stairs))

        # Size of the structure.
        self.HEIGHT = d
        # Total width of the structure.
        Base.WIDTH = a+b+c
        
    ###########################################################################
    # MOTION FUNCTION
    ###########################################################################
    # Note for all motion functions:
    # When a given motion can not be completed for any cause (wheel collision
    # or wheel pair unstable), the corresponding function does not perform the
    # required motion, and returns the error distance, that is, if we call
    # the same function again subtracting the distance returned, the motion
    # now can be completed, and the structure will be set at the limit. In
    # fact, the distance returned is of the opposite sign, so that, the correct
    # distance will be the required distance plus the distance returned by the
    # function.
    
    def check_position(self):
        """General function to check the validity of the current position.
        
        After any structure motion, the position of the structure MUST be
        checked, since the functions performing the motion do not do any check.
        For this reason, this function must be called after any motion.
        The function returns two dictionaries with the following keys:
        - Dictionary 1 (for wheel collisions or actuators reaching the further
            than either limit.
            res: If True, the position is valid in terms of wheel or actuator
                collisions. In this case, only this key exists in the
                dictionary. In case of False, include these other keys:
            ver: Vertical distance for the error. That is, if we call again
                the same function adding the error returned in this key to the
                original distance, the function will succeed.
            hor: Similar to vertical, but for horizontal distances.
            act: When both the wheel collides vertically and the actuator 
                reaches its lower bound, the key 'ver' includes the largest of
                both distances. However, for some motions only the actuator
                error is needed. This value in returned in this key.
        - Dictionary 2 (for wheel pairs geting to an unstable position).
            res: If True, the position is valid in terms of wheel stability (at
                least one of the wheels of the pair is in a stable position).
                In this case, only this key exists in the dictionary. In case
                of False, include these other keys:
            dis: horizontal distance needed to advance the structure so that
                the one of the wheel pair is place back to a stable position.
        
        """

        # Check if any wheel has collided with the stairs.      
        re_res_col, re_hor_col, re_ver_col, re_act = \
                self.REAR.check_collision()
        fr_res_col, fr_hor_col, fr_ver_col, fr_act = \
                self.FRNT.check_collision()
        # Check if any pair of wheels are not stable.
        re_res_stb, re_dis_stb = self.REAR.check_stable()
        fr_res_stb, fr_dis_stb = self.FRNT.check_stable()
        
        # Look for collisions;
        if re_res_col and fr_res_col:
            # No unstabilities. Everything is OK.
            col = {'res': True}
        else:
            # There is at least one collision. Find it and return the
            # distance.
            if re_res_col and not fr_res_col:
                # Only the front pair has collided.
                hor_col = fr_hor_col
                ver_col = fr_ver_col
                act = fr_act
            elif not re_res_col and fr_res_col:
                # Only the rear pair has collided.
                hor_col = re_hor_col
                ver_col = re_ver_col
                act = re_act
            else:
                # Both pairs of wheels have collided. Returns the maximum
                # distance of both pairs.
                if abs(re_hor_col) > abs(fr_hor_col):
                    hor_col = re_hor_col
                else:
                    hor_col = fr_hor_col
                if abs(re_ver_col) > abs(fr_ver_col):
                    ver_col = re_ver_col
                else:
                    ver_col = fr_ver_col
                if abs(re_act) > abs(fr_act):
                    act = re_act
                else:
                    act = fr_act
                    
            col = {'res': False, 'ver': ver_col, 'hor': hor_col, 'act': act}
            
        # Look for unstabilities.
        if re_res_stb and fr_res_stb:
            # No unstabilities. Everything is OK.
            stb = {'res': True}
        else:
            # There is at least one unstability. Find it and return the
            # distance.
            if re_res_stb and not fr_res_stb:
                dis_stb = fr_dis_stb
            elif not re_res_stb and fr_res_stb:
                dis_stb = re_dis_stb
            else:
                # Both pairs of wheels are unstable. Returns the largest
                # distance of both pairs.
                if abs(re_dis_stb) > abs(fr_dis_stb):
                    dis_stb = re_dis_stb
                else:
                    # And viceversa.
                    dis_stb = fr_dis_stb
            stb = {'res': False, 'dis': dis_stb}
        
        return col, stb
      
    def advance(self, distance, check=True):
        """Advance the structure horizontally.
        
        Returns False if the structure can not be moved the required distance,
        for instance, because one of the wheels collides with a step. In this
        case, it also returns the distance that can be moved, that is, the
        distance of the closest wheel to the stair.
        
        Parameters:
        distance -- Horizontal distance to move (positive, move right).
        check -- If True, after performing the motion, check that the structure
          is still in a valid position. The False value is intended to place
          the structure back to a valid position after a wrong motion inside
          the own function.
        
        """
        # Update structure position
        self.shift += distance
        if not check:
            return True, 0.0
        
        # From here on, check the validity of the motion.
        col, stb = self.check_position()
        if col['res'] and stb['res']:
            # Everything is OK.
            return True, 0.0
        
        if not col['res'] and stb['res']:
            # Only collision detected.
            # Only a collision happens.
            dis = col['hor']
        elif col['res'] and not stb['res']:
            # Only unstability detected.
            # Only a unstability happens.
            dis = stb['dis']
        else:
            # Both error happens. Get the larger of them.
            # Both collision ans unstability happens. Get the largest of both
            # distances.
            if distance > 0:
                dis = min([col['hor'], stb['dis']])
            else: 
                dis = max([col['hor'], stb['dis']])
                
        # Set the structure back to its original position.
        self.advance(-distance, False)
        # Check that everything is OK again.
        col, stb = self.check_position()
        if col['res'] and stb['res']:
            return False, dis
        raise RuntimeError("Error in advance structure")
    
    def elevate(self, distance, check=True):
        """Elevate (or take down) the whole structure.
         
        Returns False if the structure can not been elevated the complete
        distance. For returned values, see advance function.
         
        Parameters:
        distance -- Vertical distance to move (positive, structure move
            upwards.
        check -- See advance function.
                     
        """
        # Elevate the structure,
        self.elevation += distance
        # and place the actuators in the correct position.
        self.REAR.shift_actuator(True, True, distance)
        self.FRNT.shift_actuator(True, True, distance)
        
        if not check:
            return True, 0.0
        
        # Check if any of the actuators has reached one of its bounds.
        col, __ = self.check_position()
        if col['res']:
            # Everything is OK.
            return True, 0.0
        
        # In case of failure, take the distance the actuators have failed.
        dis = col['ver']

        # Leave the structure in its original position.
        self.elevate(-distance, False)
        # Check that everything is OK again.
        col, __ = self.check_position()
        if col['res']:
            return False, dis
        raise RuntimeError("Error in elevate")
        
    def shift_actuator(self, index, distance, check=True):
        """Shift one actuator independently.
         
        Returns False if the actuator can not be moved the complete distance
        because the actuator reach one of its limits, or the ending wheel
        collides with the ground.
         
        Parameters:
        index -- Index of actuator (0-3)
        distance -- Distance to shift (positive, move downwards).
        cehck -- See advance function.
         
        """
        # Select the actuator to shift.
        if index == 0:
            self.REAR.shift_actuator(True, False, distance)
        elif index == 1:
            self.REAR.shift_actuator(False, True, distance)
        elif index == 2:
            self.FRNT.shift_actuator(True, False, distance)
        elif index == 3:
            self.FRNT.shift_actuator(False, True, distance)
        
        if not check:
            return True, 0.0
        
        # Check if the actuator has reached one of its bounds.
        col, stb = self.check_position()
        # The variable col is for possible collisions with the steps, or an
        # actuator reaching one of its bounds.
        # The variable stb is for checking that, after elevating one wheel,
        # the other wheel of the pair is still in a stable position.
        if not stb['res']:
            # We can not shift the actuator, because the other wheel is not
            # in the ground. For this, the error distance is the whole distance
            # required, because we can not move the actuator at all.
            dis = -distance
        elif not col['res']:
            dis = col['ver']
        else:
            return True, 0.0
        
        # Leave the actuator in its original position.
        self.shift_actuator(index, -distance, False)        
        # Check that everything is OK again.
        col,stb = self.check_position()

        if col['res'] and stb['res']:
            return False, dis
        raise RuntimeError("Error in shift actuator.")  
      
    def incline(self, distance, 
                elevate_rear=False, fix_front=False, check=True):
        """Incline the base of the structure.
         
        Returns False if the structure has not completed the whole motion due
        to any of the reasons explained within the code.
         
        Parameters:
        distance -- Vertical distance to move the exterior actuators (actuators
            0 or 3). The angle can be computed from this value and the length
            of the structure.
        elevate_rear -- If True, when inclining, the rear edge of the structure
            is elevated, while the front remains fixed, an vice versa.
        fix_front -- When inclining the structure, the gaps between wheels
            change, so that at least all wheels except one must move. If this
            parameter is True, the fixed wheel is the four one (front wheel).
            If False, the fixed one is the first (rear wheel).
        check -- See advance function.
             
        """
        # Current computations keep fixed the rear edge of the structure. To
        # change this and elevate the front edge instead, we simply have to
        # elevate the whole structure the same distance in the opposite way.
        if elevate_rear:
            self.elevation -= distance
            # Set the actuators to its new position before inclining. Note
            # that we need not check whether they are in a valid position
            # since it can happen that, even in an invalid position at this
            # step, the actuator can return back to a valid position after
            # the inclination.
            self.REAR.shift_actuator(True, True, -distance)
            self.FRNT.shift_actuator(True, True, -distance)

        # Get vertical coordinates of the outer joints to update structure
        # angle.
        __, y0 = self.REAR.REAR.JOINT.position(0)
        x3, y3 = self.FRNT.FRNT.JOINT.position(0)
        h = y3-y0
        # Update the angle taking into account the new height to lift.
        self.angle = asin( (h + distance) / self.WIDTH )
 
        # If we fix the rear wheel, the structure does not move (that is, the
        # reference frame does not move).
        # However, if we fix the front wheel, the reference frame does move,
        # and so, we need to compute that motion to leave the front wheel
        # fixed.
        x3d = 0.0
        if fix_front:
            # Compute the motion undergone by the front wheel.
            __, __, x3d, __ = self.FRNT.position(0)
            x3d -= x3
            # And move the structure in the opposite direction.
            self.shift -= x3d

        # Elevate all the actuators (except the first one that does not move)
        # the corresponding distance to get the required inclination.
        self.REAR.shift_actuator_proportional(False, True, distance)
        self.FRNT.shift_actuator_proportional(True, True, distance)
 
        if not check:
            return True, 0.0, 0.0
        
        # Check the validity of the motion.
        # TODO: When fixing front or elevating the rear wheel, it is possible
        # that this function work wrongly. Check it.
        col, stb = self.check_position()

        if col['res'] and stb['res']:
            return True, 0.0, 0.0
        elif not col['res']:
            # Check if there is a problem with an actuator.
#             if col['act'] != 0:
#                 ver = col['act']
#             else:
#                 ver = 0.0
            # NOTE: When inclining the structure, a wheel collision can never
            # happen, because the wheel do not change its height with respect
            # to the ground. For that reason, only need to check for actuator
            # collisions.
            ver = col['act']
            hor = col['hor']
        elif not stb['res']:
            hor = stb['dis']
            ver = 0.0
        else:
            raise RuntimeError("Error in incline function")
        
        # Leave the structure in its original position.
        self.incline(-distance, elevate_rear, fix_front, False)
        # Check that everything is OK again.
        col, stb = self.check_position()
        if col['res'] and stb['res']:
            return False, hor, ver
        raise RuntimeError("Error in incline function")

    # =========================================================================
    # Control functions.
    # =========================================================================
    def get_wheels_distances(self):
        """Computes the distances from a wheel to the stairs.
        
        Returns:
        - The index of the wheel to shift, that is, the wheel that is closest
            to its corresponding step:
            - 0: Rearmost wheel.
            - 3: Frontmost wheel.
        - The horizontal distance to move.
        - The vertical height for the wheel to shift.
        
        See stair.set_distances function, and getDistances.svg.
        """
        # Compute distances for the rear pair, and the front pair.
        re_id, re_hor, re_ver = self.REAR.get_wheel_distances()
        fr_id, fr_hor, fr_ver = self.FRNT.get_wheel_distances()
        # Take the minimum of both pairs.
        if re_hor < fr_hor:
            return re_id, re_hor, re_ver
        else:
            # NOTE: The index of wheel are numbered from 0. Since the rear
            # wheel is the third wheel of the structure, we have to add 2 to
            # the index returned for the pair. 
            return fr_id + 2, fr_hor, fr_ver
 
    def set_horizontal(self):
        """Returns the distances needed to place the structure in horizontal.
        
        """
        
        # Check if also any wheel need to be set to the ground.
        re_res = self.REAR.set_to_ground()
        fr_res = self.FRNT.set_to_ground()
        if re_res is not None and fr_res is not None:
            # TODO: Do this when it is possible for the simulator to shift
            # two actuators at the same time.
            raise NotImplementedError("Move two actuator")
        elif re_res is not None:
            res = {
                'wheel': re_res[0],
                'shift': -re_res[1]}
        elif fr_res is not None:
            res = {
                'wheel': fr_res[0] + 2,
                'shift': -fr_res[1]}
        else:
            res = {
                'distance': 20.0,
                'incline': -self.get_inclination(),
                'elevate': -self.get_elevation(),
                'end': True}
            # NOTE: Add 20 unit in horizontal so that the structure stops 20 units
            # far away from the edge of the last step of the stair.
        return res

    def get_inclination(self):
        """Returns the inclination of the structure.
         
        """
        # The inclination is computed as the difference in height between the
        # front and the rear joints of the structure.
        __, y0, __, __ = self.REAR.position(0)
        __, __, __, y3 = self.FRNT.position(0)
        return y3-y0
    
    def get_elevation(self):
        """Returns the elevation of the rear actuator.
        
        """
        return self.REAR.REAR.d
    # =========================================================================
    # Drawing functions.
    # =========================================================================
    # Base colors and widths.
    BASE_COLOR = (0xFF, 0x00, 0xB3)
    BASE_WIDTH = 6
 
    def draw(self, origin, image, scale, shift):
        """Draw complete wheelchair.
        """
        x1, y1, __, __ = self.REAR.position(0)
        __, __, x2, y2 = self.FRNT.position(0)
 
        cx1 = float32(scale*(origin[0]+x1))
        cy1 = float32(scale*(origin[1]-y1))
        cx2 = float32(scale*(origin[0]+x2))
        cy2 = float32(scale*(origin[1]-y2))
        cv2.line(image, (cx1, cy1), (cx2, cy2), self.BASE_COLOR,
                 self.BASE_WIDTH, cv2.LINE_AA, shift)
 
        dy1 = float32(scale*(origin[1]-y1+self.HEIGHT))
        dy2 = float32(scale*(origin[1]-y2+self.HEIGHT))
        cv2.line(image, (cx1, dy1), (cx2, dy2), self.BASE_COLOR,
                 self.BASE_WIDTH, cv2.LINE_AA, shift)
 
        self.REAR.draw(origin, image, scale, shift)
        self.FRNT.draw(origin, image, scale, shift)
            
###############################################################################
# End of file.
###############################################################################
