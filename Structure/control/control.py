"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to generate the instructions to move the structure.

List of instructions, along with its arguments:

- advance: Horizontal shift.
  - float: distance. If positive, shift forwards.
- elevate: Vertical elevation of the structure.
  - float: height: If positive, elevate; if negative, take down.
- incline: Incline structure.
  - float: height: If positive, the front of the structure is elevated.
  - bool: elevate_rear: If true, the rear of the structure is elevated.
- height: shift an actuator.
  - float: height. If positive, the wheel is moved downwards.
  - int: wheel. Index. 0: rear wheel, 1, 2: middle wheel, 3: front wheel.
- shift: combination of height and incline/elevate.
  - float: same as height.

"""

import copy
from math import isinf

from structure import base
from simulator import simulator


class ComputeTime:
    # TODO: Move to another file, and add comments.
    # This function is for Diego.
    def __init__(self, wheels, stairs, speed_data):

        self.stairs = stairs
        self.sim = simulator.Simulator(speed_data)
        self.wheels = wheels

    def compute(self, size):
        """Return the time required to complete the stairs."""

        # Build the elements.
        str_aux = base.Base(size, self.wheels, self.stairs)

        total_time = 0
        # Make a loop until the structure reaches the end of the stair.
        while True:
            instruction, str_aux = next_instruction(str_aux)
            total_time += self.sim.compute_iterations(instruction)
            if instruction.get('end', False):
                break
        # Return the total number of iterations needed.
        return total_time


def finish_stair(structure):
    """Generate the last instruction before finish the program
    
    Before finish the program, set the structure to its initial position
    that is, all the wheels in the ground, and all the actuator to this
    lower position.
    
    Returns the instruction to finish the program. This instruction includes
    the key 'end' that informs the simulator that the structure has reached
    the end of the stair.
    """

    instruction = {}
    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)

    # Check if all the wheel arte on the ground.
    re, fr = st_aux.set_horizontal()
    # In case any wheel is not on the ground, also generate the instruction to
    # take the wheel to the ground.
    if re[0] is not None:
        # Check it for the rear pair, and take the wheel to the ground.
        res_shf = st_aux.shift_actuator(re[0], -re[1])
        if not res_shf:
            raise RuntimeError
    if fr[0] is not None:
        # And do the same for the fornt pair.
        res_shf = st_aux.shift_actuator(fr[0] + 2, -fr[1])
        if not res_shf:
            raise RuntimeError

    elevation = st_aux.get_elevation()
    # Get the current inclination and elevation of the structure.
    inclination = st_aux.get_inclination()
    # And set the structure to the initial position by moving it in the
    # opposite direction.
    # First for the inclination.
    res_inc = st_aux.incline(-inclination)
    if not res_inc:
        if not st_aux.advance(res_inc.horizontal):
            raise RuntimeError
        if not st_aux.incline(res_inc.horizontal):
            raise RuntimeError
    # And now for the elvation. Note that these instruction must be executed
    # after the wheels have been taken to the ground, in the other case, the
    # motion would not be possible.
    res_elv = st_aux.elevate(-elevation)
    if not res_elv:
        raise RuntimeError
    
    # Generate the complete instruction.
    instruction["incline"] = -inclination
    instruction["elevate"] = -elevation
    # Generate the instructions for the wheels. Note that we have to use both
    # the wheel and wheel_aux keys, although in this special case, both have
    # the same meaning, but this is not important in this case.
    if re[0] is not None:
        instruction["wheel"] = re[0]
        instruction["height"] = fr[0]
        instruction["shift"] = \
            st_aux.get_actuators_position(re[0]) - \
            structure.get_actuators_position(re[0])
    if fr[0] is not None:
        instruction["wheel_aux"] = fr[0] + 2
        instruction["shift_aux"] = \
            st_aux.get_actuators_position(fr[0] + 2) - \
            structure.get_actuators_position(fr[0] + 2)
    # TODO: It is possible that one of the two motion for the wheels can not
    # be completed. Correct this.
    # Add the end key to inform the simulator that we are done.
    instruction["end"] = True
    return instruction, st_aux


def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.

    The function gets the distances from each wheel of the structure to the
    stair (which is stores in the own structure), and generate the list of
    instructions to perform the next step.

    Returns a dictionary with the instructions to perform.

    """
    # Get the distances each wheel is with respect to its closest step.
    wheel, hor, ver, wheel_aux, ver_aux = structure.get_wheels_distances()

    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. Here, we check whether we have already reached the end of the
    # structure, and so, we have to finish the program.
    if isinf(hor):
        return finish_stair(structure)

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)

    # Simulate the horizontal shift, to check that it is correct.
    res_adv = st_aux.advance(hor)
    if not res_adv:
        # This error should not happen, but if so, check if it can be complete
        # correcting the error distance detected.
        if not st_aux.advance(hor+res_adv.horizontal):
            raise RuntimeError("Error in control module")
    # Add the distance to move to the instruction. In general, the second term
    # of the sum will be 0.
    instruction = {"advance": hor+res_adv.horizontal}

    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    res_shf = st_aux.shift_actuator(wheel, -ver)
    instruction["wheel"] = wheel
    instruction["height"] = -ver
    if not res_shf:
        # If the actuator can not be sift, we have to make room for the
        # actuator to compete the motion. This action depends on the index
        # of the actuator. Continue reading the code:
        #######################################################################
        if wheel == 3:
            # Front actuator. In this case, the algorithm incline the
            # structure.
            # TODO: Check when inclining the structure and a collision raises.
            res_inc = st_aux.incline(res_shf.central)
            instruction['incline'] = res_shf.central
            if not res_inc:
                # In this case, the structure can not be incline because the
                # front actuator can not complete the motion. To solve this,
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
            res_shf = st_aux.shift_actuator(wheel, -ver)
            # Except if after the inclination the front has moved eough to
            # collide with the step. In this case, move the structure the
            # value returned, and try if it works.
            if not res_shf:
                if not st_aux.advance(res_shf.horizontal):
                    raise RuntimeError("Error in control module")
                instruction['advance'] += res_shf.horizontal
                if not st_aux.shift_actuator(wheel, -ver):
                    raise RuntimeError("Error in control module")
        #######################################################################
        elif wheel == 2:
            # Second actuator.
            # Try to elevate the structure, to test if there is enough room
            # for the actuator to complete the motion.
            res_elv = st_aux.elevate(res_shf.central)
            instruction["elevate"] = res_shf.central
            if not res_elv:
                # Not enough space for the actuator.
                # First strategy to make more space: try to incline from the
                # rear part. This would be successful for positive step, since
                # the structure tend to be low, and so, this motion is
                # generally posible.
                if not st_aux.incline(-res_elv.rear, None, True):
                    raise ValueError("Motion can not be completed")
                instruction["incline"] = -res_elv.rear
                instruction["elevate_rear"] = True
                # Once we have complete the inclination, we surely have sapce
                # enough to elevate the structure.
                res_elv = st_aux.elevate(res_shf.central)
                if not res_elv:
                    raise ValueError("Motion can not be completed")
            # However, it is still possible that we have not enough space. So,
            # check if we now can elevate the structure and so, make enough
            # space for the actuator to complete the motion.
            res_shf = st_aux.shift_actuator(wheel, -ver)
            if not res_shf:
                if not st_aux.shift_actuator(wheel, -ver+res_shf.actuator):
                    raise ValueError("Motion can not be completed")
                # Still not enough space. This happens when the actuator that
                # block the motion is a central one. In this case, we have not
                # have an instruction to accurate compute the height required,
                # but this instruction computes a height always greater than
                # the needed,so that for a general case, is always valid.
                # TODO: Make a grphic of all this.
                res_elv = st_aux.elevate(-res_shf.actuator)
                if res_elv:
                    raise ValueError("Motion can not be completed")
                if not st_aux.shift_actuator(wheel, +ver-res_shf.actuator):
                    raise ValueError("Motion can not be completed")
                if not st_aux.incline(res_elv.rear, None, True):
                    raise ValueError("Motion can not be completed")
                instruction["incline"] += res_elv.rear
                if not st_aux.elevate(2*res_shf.actuator):
                    raise ValueError("Motion can not be completed")
                instruction["elevate"] += 2*res_shf.actuator
                res = st_aux.shift_actuator(wheel, -ver)
                if not res:
                    # If not possible, generate an instruction to only elevate
                    # and incline the structure,m and expect that in the next
                    # iteration the problem will be solved.
                    instruction['height'] = 0.0
                    instruction['advance'] = 0.0

        #######################################################################
        elif wheel == 1:
            # For actuator 1, the proccess is similar than for actuator 2, but
            # since it generates less problems, this code has not been
            # developed as it should yet.
            res_elv = st_aux.elevate(res_shf.central)

            instruction["elevate"] = res_shf.central-res_elv.rear
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(wheel, -ver):
                raise RuntimeError("Error in control module")
        #######################################################################
        elif wheel == 0:
            res_inc = st_aux.incline(-res_shf.central, None, True)
            if not res_inc:
                # If the inclination fails, one less likely cause is that,
                # when inclining, one of the wheel other than then rear one,
                # can collide. In this case, correct this distance, and try
                # again.
                if res_inc.horizontal != 0.0:
                    if not st_aux.advance(res_inc.horizontal):
                        raise RuntimeError("Error in control module")
                    instruction['advance'] += res_inc.horizontal
                # In this point, we can ensure that there will be no problem
                # with horizontal collisions. Try again.
                res_inc = st_aux.incline(-res_shf.central, None, True)
                if not res_inc:
                    # If there is still problems, in this case is because there
                    # is no room for the actuator to move.
                    # Try to elevate the structure the distance remaining.
                    if not st_aux.elevate(res_inc.rear):
                        raise ValueError("Motion can not be completed")
                    instruction["elevate"] = res_inc.rear
                    if not st_aux.incline(
                            -res_shf.central+res_inc.rear, None, True):
                        raise RuntimeError("Error in control module")
            instruction["incline"] = -res_shf.central+res_inc.rear
            instruction["elevate_rear"] = True
            # If succeeded, the actuator can now be shifted.
            if not st_aux.shift_actuator(wheel, -ver):
                raise RuntimeError("Error in control module")
    # Get the shift of each actuator. This value is needed in case we have to
    # return the actual shift of the actuator, not the shift after the
    # elevation/inclination.
    # This value con be computed from the diference in shift for the actuator
    # at the current position minus the same shift in the initial position.
    instruction["shift"] = \
        st_aux.get_actuators_position(wheel) - \
        structure.get_actuators_position(wheel)

    # Check whether we can also shift another wheel in the other pair.
    instruction["wheel_aux"] = wheel_aux
    # Check if the actuator can complete the whole motion.
    res_aux = st_aux.shift_actuator(wheel_aux, -ver_aux)
    if not res_aux:
        # If not, check if it can complete the whole motion minus the distance
        # of error (this motion should work always).
        if st_aux.shift_actuator(wheel_aux, -ver_aux+res_aux.actuator):
            instruction["shift_aux"] = \
                st_aux.get_actuators_position(wheel_aux) - \
                structure.get_actuators_position(wheel_aux)
        else:
            del instruction["wheel_aux"]
    else:
        instruction["shift_aux"] = \
            st_aux.get_actuators_position(wheel_aux) - \
            structure.get_actuators_position(wheel_aux)

    return instruction, st_aux


def manual_control(key_pressed, simulator):
    """Function to convert a key to a instruction.

    """
    if key_pressed == ord('4'):
        command = {'advance': -simulator.speed_wheel}
    elif key_pressed == ord('6'):
        command = {'advance': +simulator.speed_wheel}
    elif key_pressed == ord('2'):
        command = {'elevate': -simulator.speed_elevate_dw}
    elif key_pressed == ord('8'):
        command = {'elevate': +simulator.speed_elevate_up}
    elif key_pressed == ord('q'):
        command = {'wheel': 0, 'height': -simulator.speed_actuator_up}
    elif key_pressed == ord('a'):
        command = {'wheel': 0, 'height': +simulator.speed_actuator_dw}
    elif key_pressed == ord('w'):
        command = {'wheel': 1, 'height': -simulator.speed_actuator_up}
    elif key_pressed == ord('s'):
        command = {'wheel': 1, 'height': +simulator.speed_actuator_dw}
    elif key_pressed == ord('e'):
        command = {'wheel': 2, 'height': -simulator.speed_actuator_up}
    elif key_pressed == ord('d'):
        command = {'wheel': 2, 'height': +simulator.speed_actuator_dw}
    elif key_pressed == ord('r'):
        command = {'wheel': 3, 'height': -simulator.speed_actuator_up}
    elif key_pressed == ord('f'):
        command = {'wheel': 3, 'height': +simulator.speed_actuator_dw}
    ###########################################################################
    elif key_pressed == ord('t'):
        command = {'incline': +simulator.speed_incline_up,
                   'elevate_rear': False}
    elif key_pressed == ord('g'):
        command = {'incline': -simulator.speed_incline_dw,
                   'elevate_rear': False}
    elif key_pressed == ord('y'):
        command = {'incline': +simulator.speed_incline_dw,
                   'elevate_rear': True}
    elif key_pressed == ord('h'):
        command = {'incline': -simulator.speed_incline_up,
                   'elevate_rear': True}
    ###########################################################################
    else:
        command = {}
    return command

###############################################################################
# End of file.
###############################################################################
