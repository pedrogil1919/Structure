'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to generate the instructions to move the structure.

List of instructions, along with its arguments:

- distance: Horizontal shift.
  - float: distance. If positive, shift forwards.
- elevate: Vertical elevation of the structure.
  - float: height: If positive, elevate; if negative, take down.
- incline: Incline structure.
  - float: height: If positive, structure front elevate.
  - bool: fix_front: If true, fix the front wheel when inclining.
  - bool: elevate_rear: If true, the structure rear is shift.
- shift: Shift an actuator.
  - float: height. If positive, the wheel is moved downwards.
  - int: wheel. Index. 0: rear wheel, 1, 2: middle wheel, 3: front wheel.

'''

import copy
from math import isinf

# Distance margin
# MARGIN = 2.0

def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.
    
    Returns a dictionary with the instructions to perform. 
    
    """
    # Get the distances each wheel is with respect to its closest step.
    ac_id, hor, ver = structure.get_wheels_distances()
    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. In this code, we check if we have already reached the end of the
    # structure, and so, we have to finish the program.
    if isinf(hor):
        # Before finish the program, set the structure to its initial position
        # that is, all the wheel in the ground, and all the actuator to this
        # lower position.
        ac_id, ver = structure.set_horizontal()
        if ac_id is None:
            return {
                'distance': 20.0,
                'incline': -structure.get_inclination(),
                'elevate': -structure.get_elevation(),
                'end': True}
        else:
            hor = 0.0
             
    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    # Simulate the horizontal shift, to check that it is correct.
    res_adv = st_aux.advance(hor)
    if not res_adv:
        if not st_aux.advance(hor+res_adv.horizontal):
            raise RuntimeError("Error in control module")
    # Add the distance to move to the instruction.
    instruction = {"distance": hor+res_adv.horizontal}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    res_shf = st_aux.shift_actuator(ac_id, -ver)
    instruction["wheel"] = ac_id
    instruction["shift"] = -ver
    if not res_shf:
        # If the actuator can not be sift, we have to make room for the
        # actuator to compete the motion. This action depends on the index
        # of the actuator. Continue reading the code:
        #######################################################################
        if ac_id == 3:
            # Front actuator. In this case, the algorithm incline the
            # structure.
            # TODO: Check when inclining the structure and a collision raises.
            res_inc = st_aux.incline(res_shf.central)
            if not res_inc:
                # In this case, the structure can not be incline because
                # an actuator can not complete the motion. To solve this,
                # if possible, elevate the structure the distance left.
                if not st_aux.elevate(res_inc.central):
                    raise ValueError("Motion can not be completed")
                instruction["elevate"] = res_inc.central
                # If succeeded, the inclination can now be completed.
                # TODO: Review this instruction
                if not st_aux.incline(res_shf.central-res_inc.central):
                    raise RuntimeError("Error in control module")
            instruction['incline'] = res_shf.central-res_inc.central
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(ac_id, -ver):
                raise RuntimeError("Error in control module")
        #######################################################################
        elif ac_id == 2:
            res_elv = st_aux.elevate(res_shf.central)
#             if not res:
#                 
#                 fix = (height < 0)
#                 if ac_id == 2:
#                     h_aux = front
#                 else:
#                     h_aux = rear
#                 res, __, __, __, __ = st_aux.incline(-h_aux, True, fix)
#                 if not res:
#                     raise NotImplementedError("Can not elevate.")
#                 # Since the simulator perform first the elevate function, in
#                 # this case it is necessary that the simulator performs in the
#                 # opposite order. With this key, we inform of that.
#                 instruction["incline_prev"] = -h_aux
#                 instruction["elevate_rear"] = True
#                 instruction["fix_front"] = fix
#                 res, __, __, __ = st_aux.elevate(height-incline)
#                 if not res:
#                     raise NotImplementedError("Can not elevate.")
            instruction["elevate"] = res_shf.central-res_elv.rear
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(ac_id, -ver):
                raise RuntimeError("Error in control module")
        #######################################################################
        elif ac_id == 1:
            res_elv = st_aux.elevate(res_shf.central)
            
            instruction["elevate"] = res_shf.central-res_elv.rear
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(ac_id, -ver):
                raise RuntimeError("Error in control module")
        #######################################################################
        elif ac_id == 0:
            res_inc = st_aux.incline(-res_shf.central, True)
            if not res_inc:
                # If the inclination fails, one less likely cause is that,
                # when inclining, one of the wheel other than then rear one,
                # can collide. In this case, correct this distance, and try
                # agina.
                if res_inc.horizontal != 0.0:
                    if not st_aux.advance(res_inc.horizontal):
                        raise RuntimeError("Error in control module")
                    instruction['distance'] += res_inc.horizontal
            res_inc = st_aux.incline(-res_shf.central, True)
            if not res_inc:                    
                if not st_aux.elevate(res_inc.rear):
                    raise ValueError("Motion can not be completed")
                instruction["elevate"] = res_inc.rear
                if not st_aux.incline(-res_shf.central+res_inc.rear, True):
                    raise RuntimeError("Error in control module")              
            instruction["incline"] = -res_shf.central+res_inc.rear
            instruction["elevate_rear"] = True
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(ac_id, -ver):
                raise RuntimeError("Error in control module")            
    return instruction

def manual_control(key_pressed, simulator):
    """Function to convert a key to a instruction.
    
    """
    if key_pressed == ord('4'):
        command = {'distance': -simulator.wheel_speed}
    elif key_pressed == ord('6'):
        command = {'distance': +simulator.wheel_speed}
    elif key_pressed == ord('2'):
        command = {'elevate': -simulator.str_dw_speed}
    elif key_pressed == ord('8'):
        command = {'elevate': +simulator.str_up_speed}
    elif key_pressed == ord('q'):
        command = {'wheel': 0, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('a'):
        command = {'wheel': 0, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('w'):
        command = {'wheel': 1, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('s'):
        command = {'wheel': 1, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('e'):
        command = {'wheel': 2, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('d'):
        command = {'wheel': 2, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('r'):
        command = {'wheel': 3, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('f'):
        command = {'wheel': 3, 'shift': +simulator.actuator_speed}
    ###########################################################################
    elif key_pressed == ord('t'):
        command = {'incline': +simulator.str_up_speed, 
                   'elevate_rear': False, 'fix_front': False}
    elif key_pressed == ord('g'):
        command = {'incline': -simulator.str_dw_speed,
                   'elevate_rear': False, 'fix_front': False}
    elif key_pressed == ord('y'):
        command = {'incline': +simulator.str_up_speed,
                   'elevate_rear': False, 'fix_front': True}
    elif key_pressed == ord('h'):
        command = {'incline': -simulator.str_dw_speed,
                   'elevate_rear': False, 'fix_front': True}
    ###########################################################################
    elif key_pressed == ord('u'):
        command = {'incline': +simulator.str_dw_speed,
                   'elevate_rear': True, 'fix_front': False}
    elif key_pressed == ord('j'):
        command = {'incline': -simulator.str_up_speed,
                   'elevate_rear': True, 'fix_front': False}
    elif key_pressed == ord('i'):
        command = {'incline': +simulator.str_dw_speed,
                   'elevate_rear': True, 'fix_front': True}
    elif key_pressed == ord('k'):
        command = {'incline': -simulator.str_up_speed,
                   'elevate_rear': True, 'fix_front': True}
    else:
        command = {}
    return command

###############################################################################
# End of file.
###############################################################################