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


def last_instruction(structure):
    """Generate the last instruction before finish the program

    Before finish the program, set the structure to its initial position
    that is, all the wheels in the ground, and all the actuator to their
    lower position.

    Returns the instruction to finish the program. This instruction includes
    the key 'end' that informs to the simulator that the structure has reached
    the end of the stair.

    """
    # Check if all the wheel are on the ground.
    re, fr = structure.set_horizontal()
    # In case any wheel is not on the ground, also generate the instruction to
    # take the wheel down to the ground.
    if fr[0] is not None:
        structure.shift_actuator(fr[0]+2, -fr[1], check=False)
#         structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if re[0] is not None:
        structure.shift_actuator(re[0], -re[1], check=False)
#         structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # Get the current inclination of the structure
    # and the current elevation.
    incline = -structure.get_inclination()
    elevate = -structure.get_elevation()
    structure.incline(incline, check=False)
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    structure.elevate(elevate, check=False)
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    col, stb = structure.check_position()
    if not col or not stb:
        raise RuntimeError
    instruction = {
        "incline": incline,
        "elevate": elevate,
        "wheel": re[0],
        "height": re[1],
        "end": True}
    return instruction


def last_instruction2(structure):
    """Generate the last instruction before finish the program

    Before finish the program, set the structure to its initial position
    that is, all the wheels in the ground, and all the actuator to their
    lower position.

    Returns the instruction to finish the program. This instruction includes
    the key 'end' that informs to the simulator that the structure has reached
    the end of the stair.

    """
    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    instruction = {}
    # In some cases, this can not be the last instruction. This flag is
    # intended to detect this fact.
    finish = True
    # Check if all the wheel are on the ground.
    re, fr = st_aux.set_horizontal()
    # In case any wheel is not on the ground, also generate the instruction to
    # take the wheel down to the ground.
    if re[0] is not None:
        # Check it for the rear pair, and take the wheel to the ground.
        res_shf_re = st_aux.shift_actuator(re[0], -re[1])
        if not res_shf_re:
            # If the actuator can not complete the whole motion, do all the
            # motion possible, and mark that this can not be the last
            # instruction.
            finish = False
            if not st_aux.shift_actuator(re[0], -re[1] + res_shf_re.actuator):
                raise RuntimeError
            # Generate the instructions to move the wheel.
        instruction["wheel"] = re[0]
        instruction["height"] = re[1] - res_shf_re.actuator
    if fr[0] is not None:
        # And do the same for the front pair.
        res_shf_fr = st_aux.shift_actuator(fr[0] + 2, -fr[1])
        if not res_shf_fr:
            finish = False
            if not st_aux.shift_actuator(fr[0], -fr[1] + res_shf_fr.actuator):
                raise RuntimeError
        instruction["wheel_aux"] = fr[0] + 2

    # Get the current inclination of the structure.
    inclination = st_aux.get_inclination()
    # and the current elevation.
    elevation = st_aux.get_elevation()
    # And set the structure to the initial position by moving it in the
    # opposite direction.
    # First for the inclination.
    res_inc = st_aux.incline(-inclination)
    if not res_inc:
        # Also can happen the the structure can not correct its inclination.
        finish = False
        if not st_aux.incline(-inclination + res_inc.front):
            raise RuntimeError
    instruction["incline"] = -inclination + res_inc.front
    # And now for the elevation.
    res_elv = st_aux.elevate(-elevation)
    if not res_elv:
        finish = False
        if not st_aux.elevate(-elevation + res_elv.actuator):
            raise RuntimeError
    instruction["elevate"] = -elevation + res_elv.actuator
    # Generate the shift for the wheels. Note that we have to use both
    # the wheel and wheel_aux keys, although in this special case, both have
    # the same meaning, but this is not important in this case.
    if re[0] is not None:
        instruction["shift"] = \
            st_aux.get_actuators_position(re[0]) - \
            structure.get_actuators_position(re[0])
    if fr[0] is not None:
        instruction["shift_aux"] = \
            st_aux.get_actuators_position(fr[0] + 2) - \
            structure.get_actuators_position(fr[0] + 2)
    # Add the end key to inform the simulator that we are done. Note that this
    # can be False, indicating that this is not the last instruction.
    instruction["end"] = finish
    return instruction, st_aux


def make_room_wheel3(structure, height):
    # If the motion can not be completed, can only be due to the
    # second actuator. The parameter rear indicates the height to
    # incline the structure from the rear to allow the inclination
    # to complete.
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_elv = structure.elevate(height, margin=False)
    if res_elv:
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not structure.incline(-res_elv.rear, elevate_rear=True, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not structure.elevate(height, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    return -res_elv.rear, height+res_elv.rear


def make_room_wheel2(structure, height):
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.FRNT.REAR.d+height
    if height < 0:
        total -= structure.FRNT.REAR.LENGTH

    res_shf = structure.shift_actuator(2, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(res_shf.front, margin=False)
    if res_inc:
        return res_shf.front, 0
    # TODO: The value to elevate the structure is not this one. This is
    # intended to be larger than the needed, so in this case, a exception will
    # never be raised. Hoever, try to compute the exact value.
    if not structure.incline(-2*res_inc.rear, elevate_rear=True, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_tst = structure.incline(total, margin=False)
    if res_tst:
        raise RuntimeError
    if structure.incline(total+res_tst.front, margin=False):
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        return total+res_tst.front-2*res_inc.rear, 2*res_inc.rear
    raise RuntimeError


def make_room_wheel1(structure, height):
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.REAR.FRNT.d+height
    if height < 0:
        total -= structure.FRNT.REAR.LENGTH

    res_shf = structure.shift_actuator(1, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(-res_shf.rear, elevate_rear=True, margin=False)
    if res_inc:
        return -res_shf.rear, res_shf.rear
    # TODO: This has not been checked.
    # TODO: The value to elevate the structure is not this one. This is
    # intended to be larger than the needed, so in this case, a exception will
    # never be raised. Hoever, try to compute the exact value.
    if not structure.incline(-2*res_inc.front, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_tst = structure.incline(total, elevate_rear=True, margin=False)
    if res_tst:
        raise RuntimeError
    if structure.incline(total+res_tst.rear, margin=False):
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        return total+res_tst.rear-2*res_inc.front, 2*res_inc.rear
    raise RuntimeError


def make_room_wheel0(structure, height):
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.REAR.REAR.d+height
    if height < 0:
        total -= structure.REAR.REAR.LENGTH

    res_shf = structure.shift_actuator(0, -total, margin=False)
    if res_shf:
        structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(-res_shf.actuator,
                                elevate_rear=True, margin=False)
    if res_inc:
        return -res_shf.actuator, res_shf.actuator
#     # TODO: The value to elevate the structure is not this one. This is
#     # intended to be larger than the needed, so in this case, a exception will
#     # never be raised. Hoever, try to compute the exact value.
#     if not structure.incline(-2*res_inc.front, margin=False):
#         raise RuntimeError
# #     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
#     res_tst = structure.incline(total, elevate_rear=True, margin=False)
#     if res_tst:
#         raise RuntimeError
#     if structure.incline(total+res_tst.rear, margin=False):
#         structure.GRAPHICS.draw(structure.STAIRS, structure, False)
#         return total+res_tst.rear-2*res_inc.front, 2*res_inc.rear
    raise RuntimeError


def compute_instruction(structure, wheel, hor, ver):
    """Generate the vlues for the next instruction """

    instruction = {"complete": True}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    instruction["wheel"] = wheel
    instruction["height"] = -ver
    res_shf = structure.shift_actuator(
            instruction["wheel"], instruction["height"], margin=False)
    if not res_shf:
        # If the actuator can not be sifted, we have to make room for the
        # actuator to compete the motion. This action depends on the index
        # of the actuator. The height to consider is given by parameter
        # "central". Continue reading the code:
        #######################################################################
        if wheel == 3:
            # Front actuator. In this case, the algorithm incline the
            # structure.
            instruction['incline'] = res_shf.central
            res_inc = structure.incline(instruction["incline"], margin=False)
            if not res_inc:
                # Not enough space for the actuator:
                # First step: elevate the structure the height that is
                # actually possible.
                instruction["incline"] += res_inc.front
                if not structure.incline(instruction["incline"], margin=False):
                    raise RuntimeError
                # And try to make more space for the actuator to complete the
                # motion.
                inc, elv = make_room_wheel3(structure, -res_inc.front)
                instruction["incline"] += inc
                instruction["elevate"] = elv
        elif wheel == 2:
            # Second actuator. I this case, the algorithm elevate the
            # structure. In this case, the algorithm elevate the structure.
            instruction["elevate"] = res_shf.central
            res_elv = structure.elevate(instruction["elevate"], margin=False)
            if not res_elv:
                # Not enough space for the actuator:
                # First step: elevate the structure the height that is
                # actually possible.
                instruction["elevate"] += res_elv.central
                if not structure.elevate(instruction["elevate"], margin=False):
                    raise RuntimeError
                # And try to make more space for the actuator to complete the
                # motion.
                inc, elv = make_room_wheel2(structure, -res_elv.central)
                instruction["incline"] = inc
                instruction["elevate"] += elv
        elif wheel == 1:
            # Same as wheel 2.
            # Second actuator. I this case, the algorithm elevate the
            # structure. In this case, the algorithm elevate the structure.
            instruction["elevate"] = res_shf.central
            res_elv = structure.elevate(instruction["elevate"], margin=False)
            if not res_elv:
                # Not enough space for the actuator:
                # First step: elevate the structure the height that is
                # actually possible.
                instruction["elevate"] += res_elv.central
                if not structure.elevate(instruction["elevate"], margin=False):
                    raise RuntimeError
                # And try to make more space for the actuator to complete the
                # motion.
                inc, elv = make_room_wheel1(structure, -res_elv.central)
                instruction["incline"] = inc
                instruction["elevate"] += elv
        elif wheel == 0:
            # Same as wheel 1.
            # Second actuator. I this case, the algorithm elevate the
            # structure. In this case, the algorithm elevate the structure.
            instruction["elevate"] = res_shf.central
            res_elv = structure.elevate(instruction["elevate"], margin=False)
            if not res_elv:
                # Not enough space for the actuator:
                # First step: elevate the structure the height that is
                # actually possible.
                instruction["elevate"] += res_elv.central
                if not structure.elevate(instruction["elevate"], margin=False):
                    raise RuntimeError
                # And try to make more space for the actuator to complete the
                # motion.
                inc, elv = make_room_wheel0(structure, -res_elv.central)
                instruction["incline"] = inc
                instruction["elevate"] += elv
    # Simulate the horizontal shift, to check that it is correct.
    instruction["advance"] = hor
    res_adv = structure.advance(instruction["advance"])
    if not res_adv:
        # This error should not happen, but if so, check if it can be complete
        # correcting the error distance detected. In general, the second term
        # of the sum will be 0.
        instruction["advance"] += res_adv.horizontal
        if not structure.advance(instruction["advance"]):
            raise RuntimeError

    if not res_shf:
        if not structure.shift_actuator(instruction["wheel"],
                                        instruction["height"],
                                        margin=False):
            raise RuntimeError

    return instruction


def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.

    The function gets the distances from each wheel of the structure to the
    stair (which is stores in the own structure), and generate the list of
    instructions to perform the next step.

    Returns a dictionary with the instructions to perform.

    """
    # Get the distances each wheel is with respect to its closest step.
    wheel, hor, ver, wheel_aux, ver_aux = structure.get_wheels_distances()

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. Here, we check whether we have already reached the end of the
    # structure, and so, we have to finish the program.
    if isinf(hor):
        instruction = last_instruction(st_aux)
    else:
        instruction = compute_instruction(st_aux, wheel, hor, ver)

#     instr_aux = get_instruction(st_aux, wheel_aux, 0.0, ver_aux)

#     instruction["wheel_aux"] = wheel_aux
    # Get the shift of each actuator. This value is needed in case we have to
    # return the actual shift of the actuator, not the shift after the
    # elevation/inclination.
    # This value con be computed from the diference in shift for the actuator
    # at the current position minus the same shift in the initial position.
    instruction["shift"] = \
        st_aux.get_actuators_position(instruction["wheel"]) - \
        structure.get_actuators_position(instruction["wheel"])
#     instruction["shift_aux"] = \
#         st_aux.get_actuators_position(wheel_aux) - \
#         structure.get_actuators_position(wheel_aux)

#     # Check whether we can also shift another wheel in the other pair.
#     instruction["wheel_aux"] = wheel_aux
#     # Check if the actuator can complete the whole motion.
#     res_aux = st_aux.shift_actuator(wheel_aux, -ver_aux, margin=False)
#     if not res_aux:
#         # If not, check if it can complete the whole motion minus the distance
#         # of error (this motion should work always).
#         if st_aux.shift_actuator(wheel_aux,
#                                  -ver_aux+res_aux.actuator, margin=False):
#             instruction["shift_aux"] = \
#                 st_aux.get_actuators_position(wheel_aux) - \
#                 structure.get_actuators_position(wheel_aux)
#         else:
#             del instruction["wheel_aux"]
#     else:
#         instruction["shift_aux"] = \
#             st_aux.get_actuators_position(wheel_aux) - \
#             structure.get_actuators_position(wheel_aux)

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
