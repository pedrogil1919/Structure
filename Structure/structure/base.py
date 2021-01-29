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
        self.FRONT = ActuatorPair(
                WheelActuator(a+b, d, d+g-r3, r3, self, stairs),
                WheelActuator(a+b+c, d, d+g-r4, r4, self, stairs))

        # Size of the structure.
        self.HEIGHT = d
        # Total width of the structure.
        Base.WIDTH = a+b+c
        
    def advance(self, distance):
        """Advance the structure horizontally.
        
        Returns False if the structure can not be moved the required distance,
        for instance, because one of the wheels collides with a step. In this
        case, it also returns the distance that can be moved, that is, the
        distance of the closest wheel to the stair.
        
        Parameters:
        distance -- Horizontal distance to move (positive, move right).
        
        """
        # Update structure shift
        self.shift += distance
        
        # Move both pairs of wheels, and check if there is any error in the
        # motion.
        re_res, re_dis = self.REAR.check_collision(distance)
        fr_res, fr_dis = self.FRONT.check_collision(distance)
        
        if re_res and fr_res:
            re_res, re_dis = self.REAR.check_stable(distance)
            fr_res, fr_dis = self.FRONT.check_stable(distance)
            if re_res and fr_res:
                return True, 0

        # Get the distance needed to place the structure back to a safe
        # position. This distance will be the largest of both distances
        # (note that one of the values can be 0).
        if distance > 0:
            # If distance is positive, the correction will be negative, so
            # that the largest value is the minimum of both.
            error = min([re_dis, fr_dis])
        else:
            # And viceversa.
            error = max([re_dis, fr_dis])
        
        # Leave the structure in its original position
        # and check that everything is OK again.
        self.shift -= distance
        re_res, __ = self.REAR.check_collision(distance)
        fr_res, __ = self.FRONT.check_collision(distance)
        if not re_res or not fr_res:            
            raise RuntimeError("Error in base.advance.")
        re_res, __ = self.REAR.check_stable(distance)
        fr_res, __ = self.FRONT.check_stable(distance)
        if not re_res or not fr_res:            
            raise RuntimeError("Error in base.advance.")
        return False, error
    
    def shift_actuator(self, actuator, distance):
        """Shift one actuator independently.
         
        Returns False if the actuator can not be moved the complete distance
        because the actuator reach one of its limits, or the ending wheel
        collides with the ground.
         
        Parameters:
        actuator -- Index of actuator (0-3)
        distance -- Distance to shift (positive, move downwards).
         
        """
        success = True
        # Update actuator position.
        if actuator == 0:
            res, val = self.REAR.shift_actuator(0, distance, True)
            if not res:
                success, __ = self.REAR.shift_actuator(0, -distance, True)
        elif actuator == 1:
            res, val = self.REAR.shift_actuator(1, distance, True)
            if not res:
                success, __ = self.REAR.shift_actuator(1, -distance, True)
        elif actuator == 2:
            res, val = self.FRONT.shift_actuator(0, distance, True)
            if not res:
                success, __ = self.FRONT.shift_actuator(0, -distance, True)
        elif actuator == 3:
            res, val = self.FRONT.shift_actuator(1, distance, True)
            if not res:
                success, __ = self.FRONT.shift_actuator(1, -distance, True)
        else:
            success = False
        if not success:
            raise ValueError("Actuator index incorrect")  
        return res, val
    
    def elevate(self, distance):
        """Elevate (or take down) the whole structure.
         
        Returns False if the structure can not been elevated the complete
        distance.
         
        Parameters:
        distance -- Vertical distance to move (positive, structure move
            upwards.
             
        """
        self.elevation += distance
        motion = [
            self.REAR.shift_actuator(0, distance, False),
            self.REAR.shift_actuator(1, distance, False),
            self.FRONT.shift_actuator(0, distance, False),
            self.FRONT.shift_actuator(1, distance, False)]
        res = [m[0] for m in motion]
        if not all(res):
            # Leave the structure in its original position.
            self.elevation -= distance
            mot_aux = [
                self.REAR.shift_actuator(0, -distance, False),
                self.REAR.shift_actuator(1, -distance, False),
                self.FRONT.shift_actuator(0, -distance, False),
                self.FRONT.shift_actuator(1, -distance, False)]
            res = [m[0] for m in mot_aux]
            if not all(res):
                raise RuntimeError("Error in elevate function.")
            # Compute the error distance.          
            error = [m[1] for m in motion if not m[0]]
            if distance > 0:
                return False, max(error)
            else:
                return False, min(error)
        return True, 0
#         # Check if all the actuator can complete the required action
#         motion = [ac.shift_actuator(distance) for ac in self.ACTUATORS]
#         res = [m[0] for m in motion]
#         if not all(res):
#             # If not all the actuators succeeded, return the structure back to
#             # its original position.
#             self.elevation -= distance
#             # Set all the actuators to its original position.
#             res = [ac.shift_actuator(-distance) for ac in self.ACTUATORS]
#             if not all(res):
#                 raise RuntimeError("Error in base.elevate.")
#             return False, self.get_closest(motion, 1, distance)
#         return True, 0
# 
    def incline_structure(self, distance, front):
        pass
#         """Perform the motion of the actuators to incline the structure.
#         
#         Returns two booleans arrays checking the validity of the action. Each
#         array has 4 elements, one for each of the structure actuator.
#         The first array checks for actuator shift errors (actuator reaching
#         either end, or wheel reaching the ground). The second element checks
#         if the wheel collides with a step, since when inclining the structure,
#         all the wheel but the fixed one moves slightly.
#         
#         Parameters: see function incline.
#         
#         """
#         # Get vertical coordinates of the outer joints to update structure
#         # angle.
#         __, y0 = self.ACTUATORS[0].JOINT.position(0)
#         x3, y3 = self.ACTUATORS[3].JOINT.position(0)
#         h = y3-y0
#         # Update the angle taking into account the new height to lift.
#         self.angle = asin((h+distance)/self.WIDTH)
# 
#         # Elevate all the actuators (except the first one that does not move)
#         # the corresponding distance to get the required inclination.
#         # NOTE: We do not get shift results, since they can change after the
#         # next instruction (see bellow). For that reason, here we only shift
#         # the actuators, and at the end of this function, we check the actual
#         # actuator state.
#         # TODO: Remove this commented code.
#         # res_shf = [ac.shift_actuator_proportional(distance)
#         #           for ac in self.ACTUATORS]
#         for ac in self.ACTUATORS:
#             ac.shift_actuator_proportional(distance)
#                    
# 
#         # If we fix the rear wheel, the structure does not move (that is, the
#         # reference frame does not move).
#         # However, if we fix the front wheel, the reference frame does move,
#         # and so, we need to compute that motion to leave the front wheel
#         # fixed.
#         x3d = 0.0
#         if front:
#             # Compute the motion undergone by the front wheel.
#             x3d, __ = self.ACTUATORS[3].JOINT.position(0)
#             x3d -= x3
#             # And move the structure in the opposite direction.
#             Base.shift -= x3d
# 
#         # Since the wheels will shift slightly when inclining the structure, we
#         # need to check if, after the motion, all the wheels are placed in a
#         # correct position.
#         res_mov = [ac.move_actuator() for ac in self.ACTUATORS]
# 
#         # If any of res_shf is false, can be due to a wheel inside before the
#         # horizontal motion due to the inclination, and could have been place
#         # in a correct position afterwards (specially when correcting after
#         # an error in a try to incline). Check its correct position here.
#         res_shf = [ac.shift_actuator(0) for ac in self.ACTUATORS]
#         # TODO: Remove this commented code.
# #         for n in range(4):
# #             if not res_shf[n][0]:
# #                 res_shf[n][0] = self.ACTUATORS[n].shift_actuator(0)
#                 
#         return res_mov, res_shf
# 
    def incline(self, distance, elevate_rear=False, fix_front=False):
        pass
#         """Incline the base of the structure, keeping it vertical.
#         
#         Returns False if the structure has not completed the whole motion due
#         to any of the reasons explained within the code.
#         
#         Parameters:
#         distance -- Vertical distance to move the exterior actuators (actuators
#             0 or 3). The angle can be computed from this value and the length
#             of the structure.
#         elevate_rear -- If True, when inclining the rear edge of the structure
#             is elevated, while the front remains fixed, an vice versa.
#         fix_front -- When inclining the structure, the gaps between wheels
#             change, so that at least all wheels except one must move. If this
#             parameter is True, the fixed wheel is the four one (front wheel).
#             If False, the fixed one is the first (rear wheel).
#             
#         """
#         # Current computations keep fixed the rear edge of the structure. To
#         # change this and elevate the front edge instead, we simply have to
#         # elevate the whole structure the same distance in the opposite way.
#         if elevate_rear:
#             self.elevation -= distance
#             for ac in self.ACTUATORS:
#                 # Set the actuators to its new position before inclining. Note
#                 # that we need not check whether they are in a valid position
#                 # since it can happen that, even in an invalid position at this
#                 # step, the actuator can return back to a valid position after
#                 # the inclination.
#                 ac.shift_actuator(-distance)
#         # Perform the motion for the actuators to incline the structure.
#         motion, shift = self.incline_structure(distance, fix_front)
#         res_mot = [m[0] for m in motion]
#         res_shf = [m[0] for m in shift]
#         # Check if all the actuators succeed.
#         if not all(res_mot) or not all(res_shf):
#             # Undo the action completely.
#             if elevate_rear:
#                 self.elevation += distance
#                 for ac in self.ACTUATORS:
#                     ac.shift_actuator(distance)
#             motion, shift = self.incline_structure(-distance, fix_front)
#             res_mot = [m[0] for m in motion]
#             res_shf = [m[0] for m in shift]
#             if not all(res_mot) or not all(res_shf):
#                 raise RuntimeError("Error in base.incline.")
#             return False
# 
#         # Finally, check if, after the inclination, the structure is still in a
#         # stable position. In this case, errors can happen when a wheel is
#         # close to the edge of a downstairs step. After inclining the
#         # structure, it can be moved beyond the edge, and be placed in an
#         # unstable position.
#         # If this happens, the inclination can not be done and we have to
#         # completely undone the action.
#         ground = [ac.ground() for ac in self.ACTUATORS]
#         # We need at least one wheel of each side of the structure in the
#         # ground.
#         if not (any(ground[0:2]) and any(ground[2:4])):
#             # Undo the action completely.
#             if elevate_rear:
#                 self.elevation += distance
#                 for ac in self.ACTUATORS:
#                     ac.shift_actuator(distance)
#             res_mov, res_shf = self.incline_structure(-distance, fix_front)
#             # Check everything is OK now and return.
#             if not all(res_mov) or not all(res_shf):
#                 raise RuntimeError("Error in structure function incline.")
#             return False
# 
#         return True
# 
#     def save_state(self, ws):
#         """Save current state to a csv file.
#         
#         """
#         # Get actual number of rows saved in the spreadsheet:
#         i = len(ws.rows)
#         # Write data:
#         ws.write(i, 0, self.ACTUATORS[0].d)
#         ws.write(i, 1, self.ACTUATORS[1].d)
#         ws.write(i, 2, self.ACTUATORS[2].d)
#         ws.write(i, 3, self.ACTUATORS[3].d)
#         ws.write(i, 4, self.angle)
#         ws.write(i, 5, self.shift)
#        
#     def get_actuator_shifts(self):
#         """Returns maximum shift for all the actuators.
#         
#         For the current structure position, get the maximum allowed shift for
#         each actuator, both in positive and in negative direction.
#         Note that the minimum value in each direction is also the maximum
#         elevation allowed for the structure.
#         
#         """
#         return [act.get_maximum_shift() for act in self.ACTUATORS]
# 
#     def get_wheels_distances(self):
#         """Return the distances from a wheel to the stairs.
#         See stair.set_distances function, and getDistances.svg.
#         """
#         return [act.get_wheel_distances() for act in self.ACTUATORS]
# 
#     def get_inclination(self):
#         """Returns the inclination of the structure.
#         
#         """
#         # The inclination is computed as the difference in height between the
#         # front and the rear joints of the structure.
#         __, y0 = self.ACTUATORS[0].JOINT.position()
#         __, y3 = self.ACTUATORS[3].JOINT.position()
#         return y3-y0
# 
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
        __, __, x2, y2 = self.FRONT.position(0)
 
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
        self.FRONT.draw(origin, image, scale, shift)
            
###############################################################################
# End of file.
###############################################################################
            