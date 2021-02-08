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
    # the same function again substracting the distance returned, the motion
    # now can be completed, and the structure will be set at the limit. In
    # fact, the distance returned is of the opposite sign, so that, the correct
    # distance will be required distance plus the distance returned by the
    # function.
    
    def check_position(self, distance):

        # Check if any wheel has collided with the stairs.      
        re_res_col, re_hor_col, re_ver_col, re_act = \
                self.REAR.check_collision(distance)
        fr_res_col, fr_hor_col, fr_ver_col, fr_act = \
                self.FRNT.check_collision(distance)
        # Check if any pair of wheels are not stable.
        re_res_stb, re_dis_stb = self.REAR.check_stable(distance)
        fr_res_stb, fr_dis_stb = self.FRNT.check_stable(distance)
        
        # Look for collisions;
        if re_res_col and fr_res_col:
            # No unstabilities. Everything is OK.
            col = {'res': True}
        else:
            # There is at least one unstability. Find it and return the
            # distance.
            if re_res_col and not fr_res_col:
                hor_col = fr_hor_col
                ver_col = fr_ver_col
                act = fr_act
            elif not re_res_col and fr_res_col:
                hor_col = re_hor_col
                ver_col = re_ver_col
                act = re_act
            else:
                # Both pairs of wheels are unstable. Returns the maximum
                # distance of both pairs.
                if distance > 0:
                    hor_col = min([re_hor_col, fr_hor_col])
                    ver_col = min([re_ver_col, fr_ver_col])
                    act = min([re_act, fr_act])
                else:
                    # And viceversa.
                    hor_col = max([re_hor_col, fr_hor_col])
                    ver_col = max([re_ver_col, fr_ver_col])
                    act = max([re_act, fr_act])
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
                # Both pairs of wheels are unstable. Returns the maximum
                # distance of both pairs.
                if distance > 0:
                    dis_stb = min([re_dis_stb, fr_dis_stb])
                else:
                    # And viceversa.
                    dis_stb =max([re_dis_stb, fr_dis_stb])
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
        # TODO: In some cases both collision and unstable wheel pair can happen
        # for two different wheels. In that case, this function will not work
        # because only check for one of then (first it checks collisions, and
        # only if no collision happens, it check for unstability). This can
        # only happens when we combine up and downstairs steps. If only up or
        # down exist, this event can never happens.
        
        # Update structure position
        self.shift += distance
        if not check:
            return True, 0.0
        
        col, stb = self.check_position(distance)
        if col['res'] and stb['res']:
            return True, 0.0
        
        if not col['res'] and stb['res']:
            # Only collision detected.
            dis = col['hor']
        elif col['res'] and not stb['res']:
            # Only unstability detected.
            dis = stb['dis']
        else:
            # Both error happens. Get the larger of them.
            if distance > 0:
                dis = min([col['hor'], stb['dis']])
            else: 
                dis = max([col['hor'], stb['dis']])
                
        # Set the structure back to its original position.
        self.advance(-distance, False)
        # Check that everything is OK again.
        col, stb = self.check_position(distance)
        
        if col['res'] and stb['res']:
            return False, dis
        raise RuntimeError("Error in advance structure")
    
    def elevate(self, distance, check=True):
        """Elevate (or take down) the whole structure.
         
        Returns False if the structure can not been elevated the complete
        distance.
         
        Parameters:
        distance -- Vertical distance to move (positive, structure move
            upwards.
        check -- See advance function.
                     
        """
        self.elevation += distance
        self.REAR.shift_actuator(True, True, distance)
        self.FRNT.shift_actuator(True, True, distance)
        
        if not check:
            return True, 0.0
        
        # Check if any actuator has reached one of its bounds.
        col, __ = self.check_position(distance)
        if col['res']:
            return True, 0.0
        dis = col['ver']

        # Leave the structure in its original position.
        self.elevate(-distance, False)
        # Check that everything is OK again.
        col, __ = self.check_position(distance)
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
         
        """
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
        col, stb = self.check_position(distance)
        if not stb['res']:
            dis = -distance
        elif not col['res']:
            dis = col['ver']
        else:
            return True, 0.0
        
        # Leave the actuator in its original position.
        self.shift_actuator(index, -distance, False)        
        # Check that everything is OK again.
        col,stb = self.check_position(distance)

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
            # Compute the mottion undergone by the front wheel.
            x3d, __ = self.FRNT.FRNT.JOINT.position(0)
            x3d -= x3
            # And move the structure in the opposite direction.
            self.shift -= x3d

        # Elevate all the actuators (except the first one that does not move)
        # the corresponding distance to get the required inclination.
        self.REAR.shift_actuator_proportional(False, True, distance)
        self.FRNT.shift_actuator_proportional(True, True, distance)
 
        if not check:
            return True, 0.0
 
        col, stb = self.check_position(distance)

        if col['res'] and stb['res']:
            return True, 0.0, 0.0
        
        # This code is similar to advance function.
        if not col['res'] and stb['res']:
            # Only collision detected.
            hor = col['hor']
            # For the horizontal case, only the actuator error is needed,
            # because no wheel error can happen when inclining.
            ver = col['act']
        elif col['res'] and not stb['res']:
            # Only unstability detected.
            hor = stb['dis']
            ver = 0.0
        else:
            ver = col['act']
            # Both error happens. Get the larger of them.
            if distance > 0:
                hor = min([col['hor'], stb['dis']])
            else: 
                hor = max([col['hor'], stb['dis']])
            
        # Set the structure back to its original position.
        self.incline(-distance, elevate_rear, fix_front, False)
        # Check that everything is OK again.
        col, stb = self.check_position(distance)
        if col['res'] and stb['res']:
            return False, hor, ver
        raise RuntimeError("Error in incline function")
    
        # Perform the motion for the actuators to incline the structure.
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

#     def incline_structure(self, distance, front):
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
# #         # Get vertical coordinates of the outer joints to update structure
# #         # angle.
# #         __, y0 = self.REAR.REAR.JOINT.position(0)
# #         x3, y3 = self.FRNT.FRNT.JOINT.position(0)
# #         h = y3-y0
# #         # Update the angle taking into account the new height to lift.
# #         self.angle = asin( (h + distance) / self.WIDTH )
# #  
# #         # Elevate all the actuators (except the first one that does not move)
# #         # the corresponding distance to get the required inclination.
# #         # NOTE: We do not get shift results, since they can change after the
# #         # next instruction (see bellow). For that reason, here we only shift
# #         # the actuators, and at the end of this function, we check the actual
# #         # actuator state.
# #         # TODO: Remove this commented code.
# #         # res_shf = [ac.shift_actuator_proportional(distance)
# #         #           for ac in self.ACTUATORS]
# #         self.REAR.shift_actuator_proportional(True, True, distance)
# #         self.FRNT.shift_actuator_proportional(True, True, distance)
# #  
#         # If we fix the rear wheel, the structure does not move (that is, the
#         # reference frame does not move).
#         # However, if we fix the front wheel, the reference frame does move,
#         # and so, we need to compute that motion to leave the front wheel
#         # fixed.
#         x3d = 0.0
#         if front:
#             # Compute the mottion undergone by the front wheel.
#             x3d, __ = self.FRNT.FRNT.JOINT.position(0)
#             x3d -= x3
#             # And move the structure in the opposite direction.
#             self.shift -= x3d
#         
# #         # Since the wheels will shift slightly when inclining the structure, we
# #         # need to check if, after the motion, all the wheels are placed in a
# #         # correct position.
# #         res_mov = [ac.move_actuator() for ac in self.ACTUATORS]
# #  
# #         # If any of res_shf is false, can be due to a wheel inside before the
# #         # horizontal motion due to the inclination, and could have been place
# #         # in a correct position afterwards (specially when correcting after
# #         # an error in a try to incline). Check its correct position here.
# #         res_shf = [ac.shift_actuator(0) for ac in self.ACTUATORS]
# #         # TODO: Remove this commented code.
# # #         for n in range(4):
# # #             if not res_shf[n][0]:
# # #                 res_shf[n][0] = self.ACTUATORS[n].shift_actuator(0)
# #                  
# #         return res_mov, res_shf
#  

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
            