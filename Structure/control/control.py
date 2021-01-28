'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

'''

def manual_control(key_pressed, simulator):

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