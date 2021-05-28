"""
Created on 5 jun. 2020

@author: pedro

Step by step simulator of structure motion.

"""

from math import ceil


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
        # Maximum speeds:
        # Horizontal speed:
        self.speed_wheel = speed_data['wheel']
        # Actuator speed without load
        self.speed_actuator_up = speed_data['actuator_up']
        self.speed_actuator_dw = speed_data['actuator_dw']
        self.speed_elevate_up = speed_data['elevate_up']
        self.speed_elevate_dw = speed_data['elevate_dw']
        self.speed_incline_up = speed_data['incline_up']
        self.speed_incline_dw = speed_data['incline_dw']
        self.sample_time = speed_data['sample_time']

    def compute_step(self, speed, sample_time):
        """Compute the distance traveled in one iteration."""

        # TODO: add here all the dynamics of the system

        distance = speed * sample_time
        return distance

    def compute_iterations(self, instruction):
        """Complete the munber of iteration to complete an instruction."""

        # Get the distances to move from the instruction.
        advance = instruction.get('advance', 0.0)
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        shift = instruction.get('shift', 0.0)
        sh_aux = instruction.get('shift_aux', 0.0)

        # Compute the number of iterations needed to complete the instruction.
        # Time required to complete the horizontal motion.
        total_time = abs(advance) / self.speed_wheel

        # Time required to complete the actuator shift (if any).
        # Main actuator.
        if shift > 0:
            shift_time = shift / self.speed_actuator_dw
        else:
            shift_time = -shift / self.speed_actuator_up
        if shift_time > total_time:
            total_time = shift_time
        # The same for the second actuator.
        if sh_aux > 0:
            shift_time = sh_aux / self.speed_actuator_dw
        else:
            shift_time = -sh_aux / self.speed_actuator_up
        if shift_time > total_time:
            total_time = shift_time

        # Time required to complete the structure elevation (if any).
        if elevate > 0:
            elevate_time = elevate / self.speed_elevate_up
        else:
            elevate_time = -elevate / self.speed_elevate_dw
        if elevate_time > total_time:
            total_time = elevate_time

        # Time required to complete the structure inclination (if any).
        if incline > 0:
            incline_time = incline / self.speed_elevate_up
        else:
            incline_time = -incline / self.speed_elevate_dw
        if incline_time > total_time:
            total_time = incline_time

        # Compute total number of iterations:
        total_iterations = ceil(total_time/self.sample_time)
        if total_iterations == 0:
            total_iterations += 1
        return total_iterations

    def simulate_step(self, structure, instruction):
        """Simulate one instruction step by step."""

        advance = instruction.get('advance', 0.0)
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        shift = instruction.get('shift', 0.0)
        wheel = instruction.get('wheel', None)
        wh_aux = instruction.get('wheel_aux', None)
        sh_aux = instruction.get('shift_aux', 0.0)

        total_iterations = self.compute_iterations(instruction)
        # Compute actual speeds based on the more restrictive one.
        speed_wheel = advance / total_iterations
        speed_actuator = shift / total_iterations
        speed_elevate = elevate / total_iterations
        speed_incline = incline / total_iterations
        speed_ac_aux = sh_aux / total_iterations
        # Compute proportional speeds for the actuator based on the amount of
        # motion when elevating and inclining.
        total_motion = abs(speed_elevate) + abs(speed_incline)
        try:
            proportional_value = abs(speed_elevate) / total_motion
        except ZeroDivisionError:
            proportional_value = 0.0

        # Build the list with all the elements equal to none but the wheel
        # that must move with the structure.
        actuator_elevate = 4*[None]
        actuator_incline = 4*[None]
        try:
            actuator_elevate[wheel] = speed_actuator * proportional_value
            actuator_incline[wheel] = speed_actuator * (1-proportional_value)
        except TypeError:
            # In case there is no wheel to move, the exception raises, so that
            # all tne elements in the list are note, which is what we need.
            pass
        try:
            actuator_elevate[wh_aux] = speed_ac_aux * proportional_value
            actuator_incline[wh_aux] = speed_ac_aux * (1-proportional_value)
        except TypeError:
            # In case there is no wheel to move, the exception raises, so that
            # all tne elements in the list are note, which is what we need.
            pass

        # NOTE: If we implement a for loop, at the end of each instruction the
        # generator is called twice, causing the simulator to perform one
        # iteration without moving.
        # for __ in range(total_iterations):
        while True:
            # perform the three types of motion included in the instruction.
            # NOTE: for discrete motions, it is possible that between
            # motions the structure can reach one of its limits, but after
            # the three motions have been performed, the structure must return
            # to a valid position. For this reason, we do not check the
            # validity of the position until the three motions have been
            # performed.
            structure.advance(speed_wheel, check=False)
            structure.incline(speed_incline, actuator_incline, check=False)
            structure.elevate(speed_elevate, actuator_elevate, check=False)
            # Is here when we must check the validity of the position.
            col, stb = structure.check_position(False)
            if not col or not stb:
                raise RuntimeError("Can not move structure.")
            total_iterations -= 1
            # Check for the end of the instruction.
            if total_iterations == 0:
                break
            yield True
        # Check for the end of the trajectory. Return false when is the last
        # instruction. This is marked with the key end in the dictionary.
        yield not instruction.get('end', False)

    def simulate_instruction(self, structure, instruction):
        """Complete a list of instructions in one step.

        """
        try:
            distance = instruction['advance']
        except KeyError:
            pass
        else:
            # Advance structure
            res = structure.advance(distance)
            if not res:
                print("Can not advance structure.", res)
        #######################################################################
        # Sometimes, the order of first elevate and then incline is
        # not possible, and have to change order. With this instruction
        # perform first the inclination and the the elevation.
        elevate_post = False
        try:
            height = instruction['elevate']
        except KeyError:
            pass
        else:
            # Elevate structure
            res = structure.elevate(height)
            if not res:
                elevate_post = True
        #######################################################################
        try:
            height = instruction['incline']
        except KeyError:
            pass
        else:
            rear = instruction.get('elevate_rear', False)
            # Incline structure
            res = structure.incline(height, None, rear)
            if not res:
                print("Can not incline structure:", res)
        #######################################################################
        if elevate_post:
            try:
                height = instruction['elevate']
            except KeyError:
                pass
            else:
                # Elevate structure
                res = structure.elevate(height)
                if not res:
                    print("Can not elevate structure:", res)
        #######################################################################

        try:
            height = instruction['height']
            wheel = instruction['wheel']
        except KeyError:
            pass
        else:
            # Shift actuator.
            res = structure.shift_actuator(wheel, height)
            if not res:
                print("Can not shift actuator:", res)
        #######################################################################
        # Check for the end of the trajectory.
        try:
            if instruction['end']:
                yield False
        except KeyError:
            pass
        #######################################################################
        yield True

###############################################################################
# End of file.
###############################################################################
