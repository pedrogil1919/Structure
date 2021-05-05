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


def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.

    The function gets the distances from each wheel of the structure to the
    stair (wich is stores in the own structure), and generate the list of
    instructions to perform the next step.

    Returns a dictionary with the instructions to perform.

    """
    # Get the distances each wheel is with respect to its closest step.
    wheel, hor, ver, wheel_aux, ver_aux = structure.get_wheels_distances()

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)

    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. here, we check whether we have already reached the end of the
    # structure, and so, we have to finish the program.
    if isinf(hor):
        # Before finish the program, set the structure to its initial position
        # that is, all the wheels in the ground, and all the actuator to this
        # lower position.
        wheel, ver = structure.set_horizontal()
        if wheel is None:
            # This happens when all the wheels are already on the ground. In
            # this case, the only instruction left is to take all the actuators
            # to its lower posision, so that the structure is horizontal and in
            # its lowest position.
            # Also, advance the structure 20 units to take it far apart from
            # the last step.
            inclination = structure.get_inclination()
            elevation = structure.get_elevation()
            res_inc = st_aux.incline(inclination)
            if not res_inc:
                if not st_aux.advance(res_inc.horizontal):
                    raise RuntimeError
                if not st_aux.incline(res_inc.horizontal):
                    raise RuntimeError
            res_elv = st_aux.incline(elevation)
            return {
                'incline': -inclination,
                'elevate': -elevation,
                'end': True}, st_aux
        else:
            # In this case, first take the wheel down to the ground, and in the
            # next iteration complete the finish step.
            hor = 0.0

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)

    # If the distance to the stair is greater than the structure lehgt, first
    # advance the structure to that distance without any other motion.
    if hor > structure.WIDTH:
        instruction = {"advance": hor-structure.WIDTH}
        if not st_aux.advance(hor-structure.WIDTH):
            raise RuntimeError
        return instruction, st_aux

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
                # agina.
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
