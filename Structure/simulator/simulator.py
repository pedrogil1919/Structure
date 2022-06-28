"""
Created on 5 jun. 2020

@author: pedro.gil@uah.es

Step by step simulator of structure motion.
"""

# from math import floor
from simulator.profiles import SpeedProfile, AccelerationProfile
from enum import Enum

# Returning value after a step simulation.


class SimulatorState(Enum):
    SimulatorOK = 1
    # An error was raised when simulating.
    SimulatorError = 2
    # instruction time is smaller than sample time (no simulation performed).
    SimulatorNoIter = 3


class DynamicValueError(ValueError):
    pass


class Simulator():
    """Class to simulate structure motion."""

    def __init__(self, dynamics_data, sample_data):
        """
        Constructor:

        Arguments:
        dynamics_data: see dynamics entry in xml.

        """
        # Check that all the values are positive.
        for val in dynamics_data.values():
            if val < 0:
                raise DynamicValueError(
                    "Dynamics values must be positive.")

        if sample_data['sample_time'] < 0:
            raise DynamicValueError(
                "Sample time must be positive.")

        # Maximum speeds:
        # Actuator speed without load
        self.speed_actuator_up = dynamics_data['actuator_up']
        self.speed_actuator_dw = dynamics_data['actuator_dw']
        # Actuator speed with load
        self.speed_elevate_up = dynamics_data['elevate_up']
        self.speed_elevate_dw = dynamics_data['elevate_dw']
        self.speed_incline_up = dynamics_data['incline_up']
        self.speed_incline_dw = dynamics_data['incline_dw']
        # Sample time, for representation purposes.
        self.sample_time = sample_data['sample_time']

        # Wheel dynamics:
        # Current speed of the structure, in horizontal direction, taking into
        # account that only dynamics in horizontal direction are considered,
        # that is, for the actuators, we consider infinite acceleration /
        # decceleration.
        self.current_speed = 0.0
        # Current speed at the end of the last instrucion.
        self.end_speed = 0.0
        # Module to compute speed profiles to take adavance of the structure
        # inertia between instructions.
        try:
            self.profile = AccelerationProfile(dynamics_data)
        except KeyError:
            self.profile = SpeedProfile(dynamics_data)
            # # Horizontal speed:
            # self.speed_wheel = dynamics_data['speed']

        # Time at which the previoius instruction finishes. This is used to
        # compute the time offset for the next instruction, since the sample
        # time does not coincide generally with the instruction length.
        self.last_time = 0.0
        self.next_time = 0.0
        self.counter = 0

    def get_counter(self):
        return self.__counter

    def set_counter(self, value):
        self.__counter = value

    def get_current_speed(self):
        return self.__current_speed

    def set_current_speed(self, value):
        self.__current_speed = value

    def get_time(self):
        return self.counter * self.sample_time

    # def print_time(self):
    #     return "%8.2f %s" % (self.get_time(), self.time_units)

    def compute_actuator_time(self, instruction):
        """Compute time required by the actuators to complete the instruction.

        The function computes the time required to perform all the motions
        except the horizontal motion, since this one must be computed using
        the dynamics of the system. For the rest of the motions, we assume
        infinite acceleration.

        The function return this value, but also stores the value in the
        instruction itself, just in case this value in needed lately, and
        so, we do not need to compute this value again.

        """
        # Check if the time is already included in the instruction.
        try:
            return instruction['actuator_time']
        except KeyError:
            pass
        # If not, compute, and also store it within the instruction, following
        # the code bellow:
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

        # Include this time in the instruction, just in case this value is
        # needed for next iterations.
        instruction['actuator_time'] = total_time
        # try:
        #     instruction['mean_speed'] = distance / total_time
        # except ZeroDivisionError:
        #     instruction['mean_speed'] = float("inf")

        return total_time

    def stop_distance(self, instruction):
        """Stop distance according to the current speed.

        Computes the total distance to travel if we start a uniformly
        deccelerated motion, and considering the current speed.
        """
        # Update current speed (see comment in function compute_time).
        self.current_speed = self.end_speed
        # Get the horizontal distance to move from the instruction.
        distance = instruction['advance']
        # When considering infinite horizontal acceleration, the total time
        # required to complete the motion simply this instruction.
        # If the profiler is None, this means that we are not considering
        # structure horizontal dynamics, and so, the time required to complete
        # the horizontal motion is the distance divided by the speed.
        try:
            __, end_speed = self.profile.end_speed_range(
                self.current_speed, distance)
            stop_distance = self.profile.distance_to_stop(end_speed)
        except Exception:
            stop_distance = 0.0
        return stop_distance

    def check_collision(self, instructions, init_speed):
        """Check if a collision can happen if starting with the speed given.

        If we have to perform a list of consecutive instructions, a collision
        can happen if the initial speed is high enough, and the decceleration
        rate is not enough to deccelerate the structure before the collision
        occurs. This function check this situation, and if the collision
        happens, return the maximum speed the structure must have at the
        begining of the set of instruction to ensure no collision (i.e. the
        resulting motion is a uniformly deccelerated motion, being the
        decceleration rate the maximum).

        A collision is the situation when the vertical motion (actuators) can
        not be completed within the minimum time the structure need to complete
        the horizontal motion. This happens when the initial speed is large
        enough and even using the maximum decceleration rate the minimum time
        required to complete the horizontal motion is larger than the time
        required by the actuators.

        The set of instructions are the instruction whose total distance is
        equal to the stop distance (see profiles.distance_to_stop). No need to
        check more instruction. The calling function must ensure that this
        condition is hold.

        """
        # Set the current speed of the structure to the speed proposed to
        # begin the set, since the objective of this loop is checking
        # that, if we end at the proposed speed, there is no collision in any
        # of the instruction in the list.
        next_speed = init_speed
        # Take control of the total distance traveled. Just in case a
        # collision is detected, we will need this distance to compute back the
        # actual initial speed.
        distance_run = 0.0
        # For each instruction, we can compute the initial speed acording with
        # the limitation of that instructions. The speed to finally return is
        # the minimum of all of them.
        return_speed = init_speed
        # Just if no instruction is given, set this variable to 0.
        end_speed = init_speed
        for next_inst in instructions:
            # The actuator time is the minimum time we have to perform the
            # instruction (the actuators can not complete the instrucion in
            # less time). If the structure need more time (i.e. can not
            # deccelerate within this time), there is a collision and so, we
            # have to reduce the end speed of the current instruction.
            next_time = self.compute_actuator_time(next_inst)
            # Get the distance to travel.
            next_distance = next_inst['advance']
            # Get the minimum speed we can reach if we make an uniformly
            # deccelerated motion.
            end_speed, __ = self.profile.end_speed_range(
                next_speed, next_distance)
            # And compute the time required to get to this speed.
            # NOTE: Both values returned by the function are equal, since in
            # this case, the speed profile is a uniformly deccelerated motion.
            __, horizontal_time = self.profile.profile_time_limits(
                next_speed, end_speed, next_distance)
            # Update the total distance completed.
            distance_run += next_distance
            if horizontal_time < next_time:
                # If the maximum time is lower than the time required by
                # the actuator to complete the motion, we have to reduce
                # the end speed of the current instruction.
                # First, we compute the mean speed the instruction must
                # be done.
                mean_speed = next_distance / next_time
                # And now, if we consider a uniformly deccelerated motion,
                # the total reduction of speed will be:
                # TODO: Export this function to profile, and not use an
                # internal variable here.
                speed_delta = 0.5 * self.profile.decceleration * next_time
                if speed_delta < mean_speed:
                    # And so, the speed at the begining of the instruction will
                    # be the mean speed plus half the reduction of speed.
                    end_speed = mean_speed - speed_delta
                else:
                    end_speed = 0.0
                # Now, we have to determine the initial speed of the whole
                # set of instructions. For that purpose, we use the total
                # distance traveled so far.
                __, init_speed = self.profile.init_speed_range(
                    end_speed, distance_run)
                # We have to return the minimum of all speed computed.
                if init_speed < return_speed:
                    return_speed = init_speed
            # The speed for the next instruction will be the end speed computed
            # for the current instruction.
            next_speed = end_speed
        if end_speed > 0.0:
            # Since the list of instructions given must cover the stop distance
            # for the current speed, at the end of the loop we must have
            # reached 0 velocity. However, when we are finishing the complete
            # stair and there is not more instructions, it is possible that we
            # do not get speed equal to 0. In this case, it is obvious that
            # the end velocity is equal to 0.
            __, init_speed = self.profile.init_speed_range(0.0, distance_run)
            if init_speed < return_speed:
                return_speed = init_speed

        return return_speed

    def compute_time(self, instruction, next_instructions):
        """Compute time required to complete the instruction.

        The computation of the time is divided into two blocks. One of them
        computes the time required by the actuators to complete the elevation
        motions, choosing the sloweest of them. These computations are done
        without taking into account the actuator dynamics, i.e., infinite
        acceleration for the linear actuator.

        The other block computes the time required to move the structure
        horizontally. In this case, the computation can use the structure
        dynamics, that is, taking into account the acceleration and
        decceleration of the motors of the structure (see dynamics module).

        When computing the horizontal distance, the function need subsquent
        instruction, to chek that, if any case, there can be a crash because
        the computed horizontal speed is larger, and the structure does not
        have enough distance to break.

        However, the number of instructions needed to check this is unknown.
        So, at first instance, the function check if it has enoguh instruction
        and if not, return False. In do, the function return True.

        The function does not return any other value. The time is stored in
        the dictionary of the instruction (both, the current instruction and
        the next instructions if possible).

        """
        # Update the current speed with the end speed computed in the previous
        # iteration.
        # Note that this variable is normally updated inside the simulation
        # function, but in some times, the simulation function is not called
        # (when the simulation time for the instruction is lower than the
        # sample time), but also we can use the program without simulating.
        # For this reason, it is safer to update the current speed here.
        self.current_speed = self.end_speed
        # For the vertical time, only the current instruction is needed.
        actuator_time = self.compute_actuator_time(instruction)
        # Get the horizontal distance to move from the instruction.
        distance = instruction['advance']
        # Compute inital stimate of the end speed.
        __, end_speed = self.profile.end_speed_range(
            self.get_current_speed(), distance)
        # Check collisions and correct end speed.
        collision_speed = self.check_collision(next_instructions, end_speed)
        # If we can not start the next instruction at the computed end speed
        # above, reduce it to the maximum possible speed.
        if collision_speed < end_speed:
            end_speed = collision_speed
        # Once the end speed is known, we can compute the total time needed to
        # complete the horizontal motion.
        horizontal_time, __ = self.profile.profile_time_limits(
            self.get_current_speed(), end_speed, distance)

        # Check the one that last more time.
        if horizontal_time > actuator_time:
            # The horizontal motion is slower than the actuator, so the total
            # time is given by the horizontal motion, and all the above
            # computations are valid.
            min_time = horizontal_time
        else:
            min_time = actuator_time
            # If the actuator fixes the total time, the horizontal speed
            # profile computed above is not valid. Compute new profile (or more
            # properly, new end speed).
            end_speed_corr = self.profile.max_end_speed(
                self.current_speed, distance, min_time)
            if end_speed_corr < end_speed:
                end_speed = end_speed_corr

        # Save the computed data in the own instruction. This information is
        # needed when simulating the motion.
        instruction['time'] = min_time
        # Calculate the speed profile for the horizontal motion.
        accelerations, intervals, speeds = self.profile.compute_profile(
            self.get_current_speed(), end_speed, distance, min_time)
        instruction['dynamics'] = {
            'accelerations': accelerations,
            'intervals': intervals,
            'speeds': speeds}
        # Save the computed end speed, to replace for the computed in
        # simulation, since due to rounding errors, can be slightly different.
        self.end_speed = end_speed

    def simulate_step(self, structure, instruction):
        """Simulate one instruction step by step.

        NOTE: This function is actually a generator, to be included in a for
        loop, so that in each iteration, the generator returns True, except
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
        # I do not know what is this for.
        if structure is None:
            yield SimulatorState.SimulatorError
        # Get the time needed to complete the instruction.
        # TODO: This value must be computed with the function "comptue_time".
        # If not done, suppose there is an empty instruction, or at least an
        # instruction that last less than the sample time.
        try:
            total_time = instruction['time']
        except KeyError:
            yield Simulator.SimulatorError

        # Update the time end for the previous instruction.
        self.last_time = self.next_time
        self.current_time = self.last_time
        # # Get the total time required to complete the current instruction.
        # total_time = self.compute_time(instruction)
        # Update the time end for the current instruction.
        self.next_time += total_time
        # From the time end, we compute the last instruction simulated.
        prev_iter = int(self.last_time / self.sample_time)
        # And from the current end time, we compute the last iteration we will
        # simulate of this instruction.
        next_iter = int(self.next_time / self.sample_time)
        # With both values, we can get the total iterations we will simulate
        # for this instruction.
        total_iter = next_iter - prev_iter
        # Note however that the total iterations to simulate can be 0. In this
        # case, we can skip the simulation for this instruction. In any case,
        # the calling function will update the position of the structure by
        # other way, so it does not matter that the structure position is not
        # updated here.
        if total_iter <= 0:
            # If the instruction time is so small that we can not simulate any
            # sample, we have to warm the calling function aboutn that.
            yield SimulatorState.SimulatorNoIter

        # Fraction of a sample time not used when completing the last iteration
        # of the previous instruction. This must be used only for the first
        # iteration of the instruction. At the end of this loop this variable
        # is set to the actual sample time of the system (see end of loop).
        sample_time = self.sample_time * (prev_iter + 1) - self.last_time

        # Get vertical displacements.
        elevate = instruction.get('elevate', 0.0)
        incline = instruction.get('incline', 0.0)
        wheel = instruction.get('main', {}).get('wheel', None)
        shift = instruction.get('main', {}).get('shift', 0.0)
        wh_aux = instruction.get('second', {}).get('wheel', None)
        sh_aux = instruction.get('second', {}).get('shift', 0.0)
        # Get horizontal displacements.
        # NOTE: For the horizontal motion, we compute an array with the motion
        # for each sample time, independently of the type of profile employed.
        time_offset = self.sample_time - sample_time
        dynamics = instruction['dynamics']
        # Use the current speed stored in the instruction instead of the
        # actual current speed of the structure (when using dynamics both are
        # the same value, but not when not using it).
        current_speed = dynamics['speeds'][0]
        __, speed, position = self.profile.plot_dynamics(
            current_speed, dynamics['accelerations'],
            dynamics['intervals'], self.sample_time, time_offset)
        motion = position[1:] - position[0:-1]
        # NOTE: If we implement a for loop, at the end of each instruction the
        # generator is called twice, causing the simulator to perform one
        # iteration without moving.
        # for __ in range(total_iterations):
        for n in range(0, total_iter):
            self.counter += 1
            # Compute actual steps based on the more restrictive one.
            step_actuator = shift / total_time * sample_time
            step_elevate = elevate / total_time * sample_time
            step_incline = incline / total_time * sample_time
            step_ac_aux = sh_aux / total_time * sample_time
            # Compute proportional speeds for the actuator based on the amount
            # of motion when elevating and inclining.
            total_motion = abs(step_elevate) + abs(step_incline)
            if total_motion == 0.0:
                proportional_value = 0.0
            else:
                proportional_value = abs(step_elevate) / total_motion

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
                # that all tne elements in the list are None, which is what we
                # need.
                pass
            try:
                actuator_elevate[wh_aux] = step_ac_aux * proportional_value
                actuator_incline[wh_aux] = step_ac_aux * \
                    (1 - proportional_value)
            except TypeError:
                # In case there is no wheel to move, the exception raises, so
                # that all tne elements in the list are None, which is what we
                # need.
                pass

            step_wheel = motion[n]
            self.current_speed = speed[n]
            # step_wheel = advance / total_time * sample_time
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
            structure_position = structure.check_position(False)
            if not structure_position:
                raise RuntimeError("Can not move structure.")
            total_iter -= 1
            # Check for the end of the instruction.
            if total_iter <= 0:
                break
            yield SimulatorState.SimulatorOK
            # Remember that the sample time is set to the time offset for the
            # first iteration of the instruction. For the rest of the
            # iterations, the sample time must be the system sample time.
            sample_time = self.sample_time

    def simulate_instruction(self, structure, instruction):
        """Complete a list of instructions in one step."""
        if instruction is None:
            return
        try:
            __ = instruction['reset']
        except KeyError:
            pass
        else:
            # Advance structure
            res = structure.reset_position()
            if not res:
                print("Can not reset structure.")

        try:
            distance = instruction['advance']
        except KeyError:
            pass
        else:
            # Advance structure
            res = structure.advance(distance)
            if not res:
                horizontal = res.horizontal()
                if not structure.advance(distance + horizontal):
                    raise RuntimeError
                print("Can not advance structure.")
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
                elevation = res.elevation()
                if not structure.elevate(height + elevation, margin=False):
                    raise RuntimeError
                elevate_post = True
        #######################################################################
        try:
            height = instruction['incline']
        except KeyError:
            pass
        else:
            fixed = instruction.get('fixed', 0)
            # Incline structure
            res = structure.incline_and_avance(
                height, None, fixed, margin=False)
            if not res:
                incline = res.inclination(fixed)
                if not structure.incline_and_avance(height + incline,
                                                    None, fixed, margin=False):
                    raise RuntimeError
                print("Can not incline structure:")
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
                    if not structure.elevate(
                            height + res.elevation(), margin=False):
                        raise RuntimeError
                print("Can not elevate structure:")
        #######################################################################

        try:
            height = instruction['height']
            wheel = instruction['wheel']
        except KeyError:
            pass
        else:
            # Shift actuator.
            res = structure.push_actuator(wheel, height, margin=False)
            if not res:
                # If the wheel collides with the step, or with one of its ends,
                # modify the height to move so that the actuator is in contact
                # with the corresponding bound.
                # if not structure.push_actuator(
                #         wheel, height + res.actuator(wheel),
                #         margin=False):
                #     raise RuntimeError
                print("Can not shift actuator:")
        #######################################################################

###############################################################################

    current_speed = property(get_current_speed, set_current_speed, None, None)
    counter = property(get_counter, set_counter, None, None)
    time = property(get_time, None, None, None)

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
