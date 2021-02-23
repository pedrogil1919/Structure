'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to generate the instructions to move the structure.

List of instructions, along with its arguments:

- advance: Horizontal shift.
  - float: distance. If positive, shift forwards.
- elevate: Vertical elevation of the structure.
  - float: height: If positive, elevate; if negative, take down.
- incline: Incline structure.
  - float: height: If positive, structure front elevate.
  - bool: fix_front: If true, fix the front wheel when inclining.
  - bool: elevate_rear: If true, the structure rear is shift.
- height: height an actuator.
  - float: height. If positive, the wheel is moved downwards.
  - int: wheel. Index. 0: rear wheel, 1, 2: middle wheel, 3: front wheel.

'''

import copy
from math import isinf

# Distance margin
# MARGIN = 2.0

def next_instruction(structure, graphics, stairs):
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
            inclination = structure.get_inclination()
            elevation = structure.get_elevation()
            if inclination < 0:
                inc_key = 'incline_prev'
            else:
                inc_key = 'incline'
            return {
                'distance': 20.0,
                inc_key: -inclination,
                'elevate': -elevation,
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
    instruction = {"advance": hor+res_adv.horizontal}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    res_shf = st_aux.shift_actuator(ac_id, -ver)
    instruction["wheel"] = ac_id
    instruction["height"] = -ver
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
            instruction['incline'] = res_shf.central
            if not res_inc:
                # In this case, the structure can not be incline because
                # an actuator can not complete the motion. To solve this,
                # if possible, elevate the structure the distance left.
                if not st_aux.elevate(res_inc.rear):
                    raise ValueError("Motion can not be completed")
                instruction["elevate"] = res_inc.rear
                # If succeeded, the inclination can now be completed.
                # TODO: Review this instruction
                if not st_aux.incline(res_shf.central-res_inc.rear):
                    raise RuntimeError("Error in control module")
            instruction['incline'] -= res_inc.rear
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(ac_id, -ver):
                raise RuntimeError("Error in control module")
        #######################################################################
        elif ac_id == 2:
            # Second actuator.
            # Try to elevate the structure, to test if there is enough room
            # for the actuator to complete the motion.
            res_elv = st_aux.elevate(res_shf.central)
            instruction["elevate"] = res_shf.central
            if not res_elv:
                # Not enough space for the actuator.
                # First strategy to make more space: try to incline from the
                # rear part. This would be successful for positive step, since
                # the structure tend to be low, and so, this motion is generaly
                # posible.
                if not st_aux.incline(-res_elv.rear, True):
                    raise ValueError("Motion can not be completed")
                instruction["incline_prev"] = -res_elv.rear
                instruction["elevate_rear"] = True
                # Once we have complete 
                res_elv = st_aux.elevate(res_shf.central)
                if not res_elv:
                    raise ValueError("Motion can not be completed")
            # Check if we now can elevate the structure and so, make enough
            # space for the actuator to complete the motion.
            res_shf = st_aux.shift_actuator(ac_id, -ver)
            if not res_shf:
                if not st_aux.shift_actuator(ac_id, -ver+res_shf.actuator):
                    raise ValueError("Motion can not be completed")

#                 graphics.draw(stairs, st_aux)
                res_elv = st_aux.elevate(-res_shf.actuator)
                if res_elv:
                    raise ValueError("Motion can not be completed")
                if not st_aux.shift_actuator(ac_id, +ver-res_shf.actuator):
                    raise ValueError("Motion can not be completed")

#                 graphics.draw(stairs, st_aux)
                if not st_aux.incline(res_elv.rear, True):
                    raise ValueError("Motion can not be completed")

#                 graphics.draw(stairs, st_aux)
                instruction["incline_prev"] += res_elv.rear
                if not st_aux.elevate(2*res_shf.actuator):
                    raise ValueError("Motion can not be completed")

#                 graphics.draw(stairs, st_aux)
                instruction["elevate"] += 2*res_shf.actuator         
                res = st_aux.shift_actuator(ac_id, -ver)
                if not res:
                    instruction['height'] = 0.0
                    instruction['advance'] = 0.0
#                     raise ValueError("Motion can not be completed")
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
#                 st_aux2 = copy.deepcopy(st_aux)
#                 st_aux2.incline(-res_shf.central, True, False, False)
#                 graphics.draw(stairs, st_aux2)
                # If the inclination fails, one less likely cause is that,
                # when inclining, one of the wheel other than then rear one,
                # can collide. In this case, correct this distance, and try
                # agina.
                if res_inc.horizontal != 0.0:
                    if not st_aux.advance(res_inc.horizontal):
                        raise RuntimeError("Error in control module")
                    instruction['advance'] += res_inc.horizontal
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
    # Get the shift of each actuator. This value is needed in case we have to
    # return the actual shift of the actuator, not the shift after the
    # elevation/inclination.
    shift_prev = structure.get_actuators_position(ac_id)
    shift_curr = st_aux.get_actuators_position(ac_id)
    instruction["shift"] = shift_curr - shift_prev  


    return instruction

def manual_control(key_pressed, simulator):
    """Function to convert a key to a instruction.
    
    """
    if key_pressed == ord('4'):
        command = {'advance': -simulator.wheel_speed}
    elif key_pressed == ord('6'):
        command = {'advance': +simulator.wheel_speed}
    elif key_pressed == ord('2'):
        command = {'elevate': -simulator.str_dw_speed}
    elif key_pressed == ord('8'):
        command = {'elevate': +simulator.str_up_speed}
    elif key_pressed == ord('q'):
        command = {'wheel': 0, 'height': -simulator.actuator_speed}
    elif key_pressed == ord('a'):
        command = {'wheel': 0, 'height': +simulator.actuator_speed}
    elif key_pressed == ord('w'):
        command = {'wheel': 1, 'height': -simulator.actuator_speed}
    elif key_pressed == ord('s'):
        command = {'wheel': 1, 'height': +simulator.actuator_speed}
    elif key_pressed == ord('e'):
        command = {'wheel': 2, 'height': -simulator.actuator_speed}
    elif key_pressed == ord('d'):
        command = {'wheel': 2, 'height': +simulator.actuator_speed}
    elif key_pressed == ord('r'):
        command = {'wheel': 3, 'height': -simulator.actuator_speed}
    elif key_pressed == ord('f'):
        command = {'wheel': 3, 'height': +simulator.actuator_speed}
    ###########################################################################
    elif key_pressed == ord('t'):
        command = {'incline': +simulator.str_up_speed, 
                   'elevate_rear': False}
    elif key_pressed == ord('g'):
        command = {'incline': -simulator.str_dw_speed,
                   'elevate_rear': False}
    elif key_pressed == ord('y'):
        command = {'incline': +simulator.str_up_speed,
                   'elevate_rear': True}
    elif key_pressed == ord('h'):
        command = {'incline': -simulator.str_dw_speed,
                   'elevate_rear': True}
    ###########################################################################
    else:
        command = {}
    return command

###############################################################################
# End of file.
###############################################################################