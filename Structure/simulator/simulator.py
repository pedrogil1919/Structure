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
            # Advance structure
            res, dis = structure.advance(instruction['distance'])
            if not res:
                print("Can not advance structure. Error:", dis)
        except KeyError:
            pass

        try:
            # Elevate structure
            res, dis = structure.elevate(instruction['elevate'])
            if not res:
                print("Can not elevate structure. Error:", dis)
        except KeyError:
            pass

        try:
            # Incline structure
            res, dis = structure.incline(instruction['incline'],
                                     instruction['elevate_rear'],
                                     instruction['fix_front'])
            if not res:
                print("Can not incline structure. Error:", dis)
        except KeyError:
            pass

        try:
            # Shift actuator.
            res, dis = structure.shift_actuator(
                    instruction['wheel'], -instruction['shift'])
            if not res:
                print("Can not shift actuator. Error:", dis)
        except KeyError:
            pass

        """
        # Check for the end of the trajectory.
        try:
            if instruction['end']:
                yield False
        except KeyError:
            pass
        """
        
###############################################################################
# End of file.
###############################################################################        
        
