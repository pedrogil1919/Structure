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

def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.
    
    Returns a dictionary with the instructions to perform. 
    
    """
    

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