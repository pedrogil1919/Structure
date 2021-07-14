"""
Created on 5 jun. 2020

@author: pedro.gil@uah.es

Step by step simulator of structure motion.
"""

from math import ceil


class Simulator():
    """Class to simulate structure motion."""

    def __init__(self, speed_data):
        """
        Constructor:

        Arguments:
        speed_data: dictionary with the following keys:
            wheel -- Structure horizontal speed, in units/frame.
            actuator -- Actuator shift speed, in units/frame.
            elevate -- Structure elevate, in units/frame.
            incline -- Structure incline, in units/frame.
            NOTE: The last three parameter are actually two parameters, one
              for going up, and the other for going down (normally, going up
              is slower than going down).
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
        # Fraction of a sample time not used when completing the last iteration
        # of an instruction (see simulate_step).
        self.remaining_time = 0.0

    def compute_step(self, speed, sample_time):
        """Compute the distance traveled in one iteration."""

        # TODO: add here all the dynamics of the system

        distance = speed * sample_time
        return distance

    def compute_iterations(self, instruction):
        """Complete the number of iteration to complete an instruction."""

        # Get the distances to move from the instruction.
        advance = instruction.get('advance', 0.0)
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        shift = instruction.get('main', {}).get('shift', 0.0)
        sh_aux = instruction.get('second', {}).get('shift', 0.0)

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

        # Compute total number of iterations.
        return total_time / self.sample_time

    def simulate_step(self, structure, instruction):
        """Simulate one instruction step by step.

        NOTE: This function is actually a generator, to be included in a for
        loop, wo that in each iteration, the generator returns True, except
        when a simulation error happens, as well as update the structure
        position, so that it can be draw inside the for loop.

        Arguments:
        structure -- Actual structure to simulate.
        instruction -- Instruction to simulate.

        Note that the instruction will normally take more than one step, and so
        the generator will iterate several times, returning True except for the
        last instruction of the complete simulation (the whole simulation, not
        the simulation of this instruction).
        """
        advance = instruction.get('advance', 0.0)
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        wheel = instruction.get('main', {}).get('wheel', None)
        shift = instruction.get('main', {}).get('shift', 0.0)
        wh_aux = instruction.get('second', {}).get('wheel', None)
        sh_aux = instruction.get('second', {}).get('shift', 0.0)

        # Get the total time required to complete the current instruction.
        total_time = self.compute_iterations(instruction)
        # NOTE: The number of iterations needed to complete an instruction is
        # an integer. However, to compute the actual time required to complete
        # the whole stair, we need to keep track of all the fractions of
        # iterations we are not using in completing the last iteration of an
        # instruction.
        total_iterations = ceil(total_time)
        if total_iterations == 0:
            return True
        # So, when the accumulated fractions are greater than 1, that means
        # that we are one iteration ahead.
        self.remaining_time += (total_iterations - total_time)
        if self.remaining_time > 1:
            # In this case, we have to complete the instruction in one less
            # iteration.
            total_iterations -= 1
            # And start to accumulate the fractions to detect the next.
            self.remaining_time -= 1

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
        actuator_elevate = 4 * [None]
        actuator_incline = 4 * [None]
        try:
            actuator_elevate[wheel] = speed_actuator * proportional_value
            actuator_incline[wheel] = speed_actuator * (1 - proportional_value)
        except TypeError:
            # In case there is no wheel to move, the exception raises, so that
            # all tne elements in the list are note, which is what we need.
            pass
        try:
            actuator_elevate[wh_aux] = speed_ac_aux * proportional_value
            actuator_incline[wh_aux] = speed_ac_aux * (1 - proportional_value)
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
        """Complete a list of instructions in one step."""
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
            res = structure.elevate(height, margin=False)
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
            res = structure.incline(height, None, rear, margin=False)
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
                res = structure.elevate(height, margin=False)
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
            res = structure.shift_actuator(wheel, height, margin=False)
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
