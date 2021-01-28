'''
Created on 28 ene. 2021

@author: pedro
'''

'''
Created on 5 jun. 2020

@author: pedro

Step by step simulator of structure motion.

'''


class Simulator():
    """Class to simulate structure motion.

    """
    def __init__(self, speed_data):
        """
        Constructor:

        Parameters:
        speed_data: dictionary with the following keys:
            wheel -- Structure horizontal speed, in units/frame.
            actuator -- Actuator shift speed (without load), in units/frame.
            elevate -- Structure elevate/incline (with load), in units/frame.
            TODO: When the structure is taken down, its speed is the
                actuator speed (no load). Check if this is correct.
        """
        self.wheel_speed = speed_data['wheel']
        self.actuator_speed = speed_data['actuator']
        self.str_up_speed = speed_data['struct_up']
        self.str_dw_speed = speed_data['struct_down']

    def simulate_instruction(self, structure, instruction):
        """Complete a list of instructions in one step.

        (for more details, see simulate_simple_step function).

        """
        try:
            if not structure.advance(instruction['distance']):
                print("Control: Error in advance function.")
        except KeyError:
            pass

        try:
            # Elevate structure
            if not structure.elevate(instruction['elevate']):
                print("Control: Can not elevate structure.")
        except KeyError:
            pass

        try:
            # Incline structure
            if not structure.incline(instruction['incline'],
                                     instruction['elevate_rear'],
                                     instruction['fix_front']):
                print("Control: Can not incline structure.")
        except KeyError:
            pass

        try:
            # Shift actuator.
            if not structure.shift_actuator(
                    instruction['wheel'], -instruction['shift']):
                print("Control: Can not shift actuator")
        except KeyError:
            pass

        # Check for the end of the trajectory.
        try:
            if instruction['end']:
                yield False
        except KeyError:
            pass
        yield True
        
###############################################################################
# End of file.
###############################################################################        
        
