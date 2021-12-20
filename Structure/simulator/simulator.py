"""
Created on 5 jun. 2020

@author: pedro.gil@uah.es

Step by step simulator of structure motion.
"""

# from math import floor


class Simulator():
    """Class to simulate structure motion."""

    def __init__(self, dynamics_data):
        """
        Constructor:

        Arguments:
        dynamics_data: see dynamics entry in xml.

        """
        # Maximum speeds:
        # Horizontal speed:
        self.speed_wheel = speed_data['wheel']
        try:
            self.acceleration = dynamics_data['acceleration']
            self.decceleration = dynamics_data['decceleration']
        except KeyError:
            pass
        # Actuator speed without load
        self.speed_actuator_up = speed_data['actuator_up']
        self.speed_actuator_dw = speed_data['actuator_dw']
        # Actuator speed with load
        self.speed_elevate_up = speed_data['elevate_up']
        self.speed_elevate_dw = speed_data['elevate_dw']
        self.speed_incline_up = speed_data['incline_up']
        self.speed_incline_dw = speed_data['incline_dw']
        # Sample time, for representation purposes.
        self.sample_time = speed_data['sample_time']

        # Time at which the previoius instruction finishes. This is used to
        # compute the time offset for the next instruction, since the sample
        # time does not coincide generally with the instruction length.
        self.last_time = 0.0
        self.curr_time = 0.0

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

        # Update the time end for the previous instruction.
        self.last_time = self.curr_time
        # Get the total time required to complete the current instruction.
        total_time = self.compute_time(instruction)
        # Update the time end for the current instruction.
        self.curr_time += total_time
        # From the time end, we compute the last instruction simulated.
        prev_iter = int(self.last_time / self.sample_time)
        # And from the current end time, we compute the last iteration we will
        # simulate of this instruction.
        next_iter = int(self.curr_time / self.sample_time)
        # With both values, we can get the total iterations we will simulate
        # for this instruction.
        total_iter = next_iter - prev_iter
        # Note however that the total iterations to simulate can be 0. In this
        # case, we can skip the simulation for this instruction. In any case,
        # the calling function will update the position of the structure by
        # other way, so it does not matter that the structure position is not
        # updated here.
        if total_iter <= 0:
            return True
        # Fraction of a sample time not used when completing the last iteration
        # of the previous instruction. This must be used only for the first
        # iteration of the instruction. At the end of this loop this variable
        # is set to the actual sample time of the system (see end of loop).
        sample_time = self.sample_time * (prev_iter + 1) - self.last_time
        # NOTE: If we implement a for loop, at the end of each instruction the
        # generator is called twice, causing the simulator to perform one
        # iteration without moving.
        # for __ in range(total_iterations):
        while True:
            # Compute actual steps based on the more restrictive one.
            step_wheel = advance / total_time * sample_time
            step_actuator = shift / total_time * sample_time
            step_elevate = elevate / total_time * sample_time
            step_incline = incline / total_time * sample_time
            step_ac_aux = sh_aux / total_time * sample_time
            # Compute proportional speeds for the actuator based on the amount
            # of motion when elevating and inclining.
            total_motion = abs(step_elevate) + abs(step_incline)
            try:
                proportional_value = abs(step_elevate) / total_motion
            except ZeroDivisionError:
                proportional_value = 0.0

            # Build the list with all the elements equal to none but the wheel
            # that must move with the structure.
            actuator_elevate = 4 * [None]
            actuator_incline = 4 * [None]
            try:
                actuator_elevate[wheel] = step_actuator * proportional_value
                actuator_incline[wheel] = step_actuator * \
                    (1 - proportional_value)
            except TypeError:
                # In case there is no wheel to move, the exception raises, so
                # that all tne elements in the list are note, which is what we
                # need.
                pass
            try:
                actuator_elevate[wh_aux] = step_ac_aux * proportional_value
                actuator_incline[wh_aux] = step_ac_aux * \
                    (1 - proportional_value)
            except TypeError:
                # In case there is no wheel to move, the exception raises, so
                # that all tne elements in the list are note, which is what we
                # need.
                pass

            # perform the three types of motion included in the instruction.
            # NOTE: for discrete motions, it is possible that between
            # motions the structure can reach one of its limits, but after
            # the three motions have been performed, the structure must return
            # to a valid position. For this reason, we do not check the
            # validity of the position until the three motions have been
            # performed.
            structure.advance(step_wheel, check=False)
            structure.incline(step_incline, actuator_incline, check=False)
            structure.elevate(step_elevate, actuator_elevate, check=False)
            # Is here when we must check the validity of the position.
            col, stb = structure.check_position(False)
            if not col or not stb:
                raise RuntimeError("Can not move structure.")
            total_iter -= 1
            # Check for the end of the instruction.
            if total_iter <= 0:
                break
            yield True
            # Remember that the sample time is set to the time offset for the
            # first iteration of the instruction. For the rest of the
            # iterations, the sample time must be the system sample time.
            sample_time = self.sample_time
        # Check for the end of the trajectory. Return false when is the last
        # instruction. This is marked with the key end in the dictionary.
        yield not instruction.get('end', False)

    def compute_horizontal_time(self, instruction):
        # TODO: add dynamics
        # Compute the number of iterations needed to complete the instruction.
        # Time required to complete the horizontal motion.
        # Get the distances to move from the instruction.
        distance = instruction.get('advance', 0.0)
        total_time = abs(distance) / self.speed_wheel
        return total_time

    def compute_actuator_time(self, instruction):
        """Compute time required by the actuators to complete the instruction.

        The function computes the time required to perform all the motions
        except the horizontal motion, since this one must be computed using
        the dynamics of the system. For the rest of the motions, we assume
        infinite acceleration.

        """
        # Get the distances to move from the instruction.
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        shift = instruction.get('main', {}).get('shift', 0.0)
        sh_aux = instruction.get('second', {}).get('shift', 0.0)

        # Compute the time for all the actuator motions, and choose the
        # grerater.
        total_time = 0.0
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

        return total_time

    def compute_time(self, instruction):
        """Compute time required to complete the instruction.

        The computation of the time is divided into two blocks. One of then
        computes the time required by the actuator to complete the elevation
        motions, choosing the greater of them. These computations are done
        without taking into account the actuator dynamics, i.e., infinite
        acceleration for the linear actuator.

        The other block computes the time required to move the structure
        horizontally. In this case, the computation uses the structure
        dynamics, that is, taking into account the acceleration and
        decceleration of the motors of the structure.

        """
        actuator_time = self.compute_actuator_time(instruction)
        horizontal_time = self.compute_horizontal_time(instruction)
        return max([actuator_time, horizontal_time])

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

    # def compute_step(self, speed, sample_time):
    #     """Compute the distance traveled in one iteration."""
    #
    #     # TODO: add here all the dynamics of the system
    #
    #     distance = speed * sample_time
    #     return distance

    # def compute_iterations(self, instruction):
    #     """Compute the number of iteration to complete an instruction."""
    #
    #     # Get the distances to move from the instruction.
    #     advance = instruction.get('advance', 0.0)
    #     elevate = instruction.get('elevate', 0.0)
    #     incline = instruction.get('incline', 0.0)
    #     shift = instruction.get('main', {}).get('shift', 0.0)
    #     sh_aux = instruction.get('second', {}).get('shift', 0.0)
    #
    #     # Compute the number of iterations needed to complete the
    #     #   instruction.
    #     # Time required to complete the horizontal motion.
    #     total_time = abs(advance) / self.speed_wheel
    #
    #     # Time required to complete the actuator shift (if any).
    #     # Main actuator.
    #     if shift > 0:
    #         shift_time = shift / self.speed_actuator_dw
    #     else:
    #         shift_time = -shift / self.speed_actuator_up
    #     if shift_time > total_time:
    #         total_time = shift_time
    #     # The same for the second actuator.
    #     if sh_aux > 0:
    #         shift_time = sh_aux / self.speed_actuator_dw
    #     else:
    #         shift_time = -sh_aux / self.speed_actuator_up
    #     if shift_time > total_time:
    #         total_time = shift_time
    #
    #     # Time required to complete the structure elevation (if any).
    #     if elevate > 0:
    #         elevate_time = elevate / self.speed_elevate_up
    #     else:
    #         elevate_time = -elevate / self.speed_elevate_dw
    #     if elevate_time > total_time:
    #         total_time = elevate_time
    #
    #     # Time required to complete the structure inclination (if any).
    #     if incline > 0:
    #         incline_time = incline / self.speed_elevate_up
    #     else:
    #         incline_time = -incline / self.speed_elevate_dw
    #     if incline_time > total_time:
    #         total_time = incline_time
    #
    #     # Compute total number of iterations.
    #     return total_time / self.sample_time
