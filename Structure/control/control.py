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

# Distance margin
# MARGIN = 2.0

def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.
    
    Returns a dictionary with the instructions to perform. 
    
    """
    ac_id, hor, ver = structure.get_wheels_distances()
    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    # Simulate the horizontal shift, to check that it is correct.
    res, err = st_aux.advance(hor)
    if not res:
        res, __ = st_aux.advance(hor+err)
        if not res:
            raise RuntimeError("Error in control module")
    # Add the distance to move to the instruction.
    instruction = {"distance": hor+err}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    res, height = st_aux.shift_actuator(ac_id, -ver)
    instruction["wheel"] = ac_id
    instruction["shift"] = -ver
    if not res:
        # If the actuator can not be sift, we have to make room for the
        # actuator to compete the motion. This action depends on the index
        # of the actuator. Continue reading the code:
        #######################################################################
        if ac_id == 3:
            # Front actuator. In this case, the algorithm incline the
            # structure.
            res, distance, elevate = st_aux.incline(height)
            if not res:
                # In this case, the structure can not be incline because
                # an actuator can not complete the motion. To solve this,
                # if possible, elevate the structure the distance left.
                res, __ = st_aux.elevate(elevate)
                if not res:
                    raise ValueError("Motion can not be completed")
                instruction["elevate"] = elevate
                # If succeeded, the inclination can now be completed.
                res, __, __ = st_aux.incline(height)
                if not res:
                    raise RuntimeError("Error in control module")
            instruction['incline'] = height
            # If succeeded, the actuator can now be shifted.
            res, __ = st_aux.shift_actuator(ac_id, -ver)
            if not res:
                raise RuntimeError("Error in control module")
        #######################################################################
        elif ac_id == 2 or ac_id == 1:
            res, __ = st_aux.elevate(height)
            if not res:
                raise NotImplementedError("Can not elevate. I think I have to \
                     incline also. Wait for an example to happen")
            instruction["elevate"] = height
            # If succeeded, the actuator can now be shifted.
            res, __ = st_aux.shift_actuator(ac_id, -ver)
            if not res:
                raise RuntimeError("Error in control module")
        elif ac_id == 0:
            res, incline = st_aux.elevate(height)
            if not res:
                res, __, __ = st_aux.incline(-incline, True)
                if not res:
                    raise ValueError("Motion can not be completed")
                instruction["incline"] = -incline
                instruction["fix_front"] = True
            res, __ = st_aux.elevate(height)
            if not res:
                raise RuntimeError("Error in control module")              
            instruction["elevate"] = height
            # If succeeded, the actuator can now be shifted.
            res, __ = st_aux.shift_actuator(ac_id, -ver)
            if not res:
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
        command = {'wheel': 0, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('a'):
        command = {'wheel': 0, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('w'):
        command = {'wheel': 1, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('s'):
        command = {'wheel': 1, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('e'):
        command = {'wheel': 2, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('d'):
        command = {'wheel': 2, 'shift': -simulator.actuator_speed}
    elif key_pressed == ord('r'):
        command = {'wheel': 3, 'shift': +simulator.actuator_speed}
    elif key_pressed == ord('f'):
        command = {'wheel': 3, 'shift': -simulator.actuator_speed}
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