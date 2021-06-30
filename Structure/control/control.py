"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to generate the instructions to move the structure.

List of instructions, along with its arguments:

- advance (float): Horizontal motion, if positive, move forwards.
- elevate (float): Vertical elevation of the structure. If positive, elevate.
- incline (foat): Incline structure. If positive, the front is elevated.
- main:  (dictionary): Driven wheel (wheel closest to its corresponding step,
    with the following keys:
  - wheel (int): Wheel to shift (0 or 1).
  - height (float): shift actuator. If positive, the wheel is moved downwards.
  - shift (float): actual shift of the actuator once the structure has been
      elevetad/inclined.
- second (dictionary): Same as main, for the wheel in the other pair.

"""

import copy
from math import isinf


def last_instruction(structure):
    """Generate the last instruction before finishing the program

    Before finish the program, set the structure to its initial position
    that is, all the wheels in the ground, and all the actuator to their
    lower position.

    Returns the instruction to finish the program. This instruction includes
    the key 'end' that informs the simulator that the structure has reached
    the end of the stair.

    """
    # Check if all the wheels are on the ground.
    re, fr = structure.set_horizontal()
    # In case any wheel is not on the ground, also generate the instruction to
    # take the wheel down to the ground.
#         structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # Instruction for the rear pair.
    if re[0] is not None:
        structure.shift_actuator(re[0], -re[1], check=False)
        actuator = {
            "wheel": re[0],
            "height": re[1]}
    else:
        actuator = {}
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # Same as above, for the front pair.
    # NOTE: In this case, there is no differece between the rear and front
    # pair. However, since we have to include this information in the main and
    # second actuator dictionary, we include one of then in the main and the
    # other in the second, although the other order will work as well.
    if fr[0] is not None:
        structure.shift_actuator(fr[0]+2, -fr[1], check=False)
        act_aux = {
            "wheel": fr[0]+2,
            "height": fr[1]}
    else:
        act_aux = {}
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
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
        "end": True}
    return instruction, actuator, act_aux


def make_room_wheel3(structure, height):
    """Function to make enough space for an actuator to complete its motion.

    When an actuator can not complete its actual motion following the
    corresponding structure motion (i.e., incline for wheel 3, elevate for the
    rest), this function computes and additional motion to make more room for
    the actuator. The function return the combination of inclination/elevation
    required for the structure.

    NOTE: This is for wheel 3. There is a corresponding function for the rest
    of the wheels.

    Parameter:
    height -- additional height the actuator must be shifted and can not be
        completed with a single motion.

    """
    # For wheel 3, if the motion can not be completed, can only be due to the
    # second actuator. Try to elevate the structure.
    res_elv = structure.elevate(height, margin=False)
    if res_elv:
        return 0, height
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # In this case, the parameter rear indicates the height to incline the
    # structure from the rear to allow the inclination to complete.
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not structure.incline(-res_elv.rear, elevate_rear=True, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not structure.elevate(height, margin=False):
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    return -res_elv.rear, height+res_elv.rear


def make_room_wheel2(structure, height):
    """See make_room_wheel3"""
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # Height is the distance we need to obtain. First, compute the total motion
    # of the actuator, which is the sum of the actual height plus the current
    # position of the actuator.
    total = structure.FRNT.REAR.d+height
    if height < 0:
        # Also, if the motion is negative (that is, the wheel must be moved
        # downwards), the total motion is the last value minus the length of
        # the actuators.
        total -= structure.HEIGHT

    # If we try to shift the actuator, obviously we get an error,
    res_shf = structure.shift_actuator(2, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise RuntimeError
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # but the value front indicates the inclination for the structure to get
    # the space requirede.
    res_inc = structure.incline(res_shf.front, margin=False)
    if res_inc:
        return res_shf.front, 0

    # If the code gets here, is because the collision is with the second
    # actuator. In this case, compute the final inclination of the structure
    # after the third actuator has been shifted.
    inclination = structure.get_inclination_central_wheels(0, height)
    # Shift temporarily the third actuator to prevent collision with this
    # actuator, instead that with the second, which is the actual one with
    # which the collision will be produced.
    structure.shift_actuator(2, -height)
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    #  Set the structure to the given inclination.
    res_inc = structure.incline(inclination, elevate_rear=True)
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not res_inc:
        raise RuntimeError
    # And elevate the structure to set it back to its actual position. To get
    # the distance to elevate, first elevate a value larger than the required.
    res_elv = structure.elevate(total)
    if res_elv:
        raise RuntimeError
    # And the distance is the previous value plus the distance of error given
    # in central value.
    if not structure.elevate(total+res_elv.central):
        raise RuntimeError
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    # Finally, set the actuator to its current position (remember that we
    # elevate this actuator previously).
    structure.shift_actuator(2, +height)
#     structure.GRAPHICS.draw(structure.STAIRS, structure, False)

    return inclination, +total+res_elv.central-inclination


def make_room_wheel1(structure, height):
    """See make_room_wheel3."""
    # NOTE: This function is not as complete as the one for wheel2 because all
    # the errors faced in that function would not be produced for this actuator
    # in a normal structure operation.
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.REAR.FRNT.d+height
    if height < 0:
        total -= structure.HEIGHT

    res_shf = structure.shift_actuator(1, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise RuntimeError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(-res_shf.rear, elevate_rear=True, margin=False)
    if res_inc:
        return -res_shf.rear, res_shf.rear
    # TODO: The code here must be similar to the one in make_room_wheel2, but
    # on the other side of the structure.
    raise RuntimeError


def make_room_wheel0(structure, height):
    """See make_room_whell3"""

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

    raise RuntimeError


def compute_instruction(structure, wheel, hor, ver):
    """Generate the values for the next instruction

    The function computes the horizontal motion, elevation and inclination of
    the structure to get the motion indicaded by the params. Note that the
    structure is modified inside this function, so, it is needed a copy of the
        original one if we need to keep it.

    Returns:
    instruction: A dictionary with the following keys (see key definition 
    above):
      - advance.
      - incline.
      - elevate.
    actuator: A dictionary with the following keys (see key definition above):
      - wheel.
      - height.

    Parameters:
    structure -- The structure in its current position. The function need the
        structure to use its geometric function to compute distances of error.
    wheel -- Wheel we need to move and ensure enough space for it to move.
    hor, ver -- Horizontal and vertical distance that the wheel need to move to
        get to its next state.

    """
    instruction = {"advance": 0}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    actuator = {
        "wheel": wheel,
        "height": -ver}
    res_shf = structure.shift_actuator(
            actuator["wheel"], actuator["height"], margin=False)
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
                # First, check i fthere is a horizontal collision.
                instruction["advance"] = res_inc.horizontal
                if not structure.advance(instruction["advance"]):
                    raise RuntimeError
                res_inc = structure.incline(instruction["incline"],
                                            margin=False)
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

    # Simulate the horizontal motion, to check that it is correct.
    instruction["advance"] += hor
    res_adv = structure.advance(instruction["advance"])
    if not res_adv:
        # This error should not happen, but if so, check if it can be complete
        # correcting the error distance detected. In general, the second term
        # of the sum will be 0.
        instruction["advance"] += res_adv.horizontal
        if instruction["advance"] == 0.0:
            raise RuntimeError
        if not structure.advance(instruction["advance"]):
            raise RuntimeError
    # Check that the actuator can now be shifted the required height.
    if not res_shf:
        if not structure.shift_actuator(actuator["wheel"],
                                        actuator["height"],
                                        margin=False):
            raise RuntimeError

    return instruction, actuator


def next_instruction(structure):
    """Generate in an automatic fashion the next instruction for the structure.

    The function gets the distances from each wheel of the structure to the
    stair (which is stores in the own structure), and generate the list of
    instructions to perform the next step.

    Returns a dictionary with the instructions to perform.

    """
    # Get the distances each wheel is with respect to its closest step.
    wheel, hor, ver, w_aux, h_aux, v_aux = structure.get_wheels_distances()

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. Here, we check whether we have already reached the end of the
    # structure, and so, we have to finish the program.
    inst_aux = {}
    act_aux = {}
    if isinf(hor):
        # Computing the las instruction before finishing the program.
        instruction, actuator, act_aux = last_instruction(st_aux)
    else:
        # Get the next instruction for the main wheel.
        instruction, actuator = compute_instruction(st_aux, wheel, hor, ver)
        # And the next instruction for the wheel in the other pair.
        # NOTE: The height to shift the second actuator must be proportional
        # to the horizontal distance this wheel must move compared with the
        # horizontal distance for the main wheel. This ensure that the motion
        # for the second actuator is more regular.
        # To prevent for a zero division, add a small amount to both horizontal
        # distances. To make it invariant to system scale, add a value
        # proportional to the size of the structure.
        k = structure.WIDTH/1000
        v_total = v_aux*(hor+k)/(h_aux+k)
        inst_aux, act_aux = compute_instruction(st_aux, w_aux, 0.0, v_total)
        # Just if the second wheel need an additional motion for the
        # structure, add this motion to the original one needed by the main
        # wheel.
        try:
            # If the second wheel does not need any additional elevation, do
            # nothing.
            elv_aux = inst_aux["elevate"]
            try:
                instruction["elevate"] += elv_aux
            except KeyError:
                # And if the main wheel does not need any elevation, but does
                # the second, set the elevation to this value.
                instruction["elevate"] = elv_aux
        except KeyError:
            pass
        try:
            # Same comment than for elevation.
            inc_aux = inst_aux["incline"]
            try:
                instruction["incline"] += inc_aux
            except KeyError:
                instruction["incline"] = inc_aux
        except KeyError:
            pass

    # Get the shift of each actuator. This value is needed in case we have to
    # return the actual shift of the actuator, not the shift after the
    # elevation/inclination.
    # This value con be computed from the diference in shift for the actuator
    # at the current position minus the same shift in the initial position.
    try:
        actuator["shift"] = \
            st_aux.get_actuator_position(actuator["wheel"]) - \
            structure.get_actuator_position(actuator["wheel"])
        instruction["main"] = actuator
    except KeyError:
        pass
    try:
        act_aux["shift"] = \
            st_aux.get_actuator_position(act_aux["wheel"]) - \
            structure.get_actuator_position(act_aux["wheel"])
        instruction["second"] = act_aux
    except KeyError:
        pass

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

# def last_instruction2(structure):
#     """Generate the last instruction before finish the program
# 
#     Before finish the program, set the structure to its initial position
#     that is, all the wheels in the ground, and all the actuator to their
#     lower position.
# 
#     Returns the instruction to finish the program. This instruction includes
#     the key 'end' that informs to the simulator that the structure has reached
#     the end of the stair.
# 
#     """
#     # Create a deep copy of the structure, to simulate all the motions computed
#     # without modifying the actual structure.
#     st_aux = copy.deepcopy(structure)
#     instruction = {}
#     # In some cases, this can not be the last instruction. This flag is
#     # intended to detect this fact.
#     finish = True
#     # Check if all the wheel are on the ground.
#     re, fr = st_aux.set_horizontal()
#     # In case any wheel is not on the ground, also generate the instruction to
#     # take the wheel down to the ground.
#     if re[0] is not None:
#         # Check it for the rear pair, and take the wheel to the ground.
#         res_shf_re = st_aux.shift_actuator(re[0], -re[1])
#         if not res_shf_re:
#             # If the actuator can not complete the whole motion, do all the
#             # motion possible, and mark that this can not be the last
#             # instruction.
#             finish = False
#             if not st_aux.shift_actuator(re[0], -re[1] + res_shf_re.actuator):
#                 raise RuntimeError
#             # Generate the instructions to move the wheel.
#         instruction["wheel"] = re[0]
#         instruction["height"] = re[1] - res_shf_re.actuator
#     if fr[0] is not None:
#         # And do the same for the front pair.
#         res_shf_fr = st_aux.shift_actuator(fr[0] + 2, -fr[1])
#         if not res_shf_fr:
#             finish = False
#             if not st_aux.shift_actuator(fr[0], -fr[1] + res_shf_fr.actuator):
#                 raise RuntimeError
#         instruction["wheel_aux"] = fr[0] + 2
# 
#     # Get the current inclination of the structure.
#     inclination = st_aux.get_inclination()
#     # and the current elevation.
#     elevation = st_aux.get_elevation()
#     # And set the structure to the initial position by moving it in the
#     # opposite direction.
#     # First for the inclination.
#     res_inc = st_aux.incline(-inclination)
#     if not res_inc:
#         # Also can happen the the structure can not correct its inclination.
#         finish = False
#         if not st_aux.incline(-inclination + res_inc.front):
#             raise RuntimeError
#     instruction["incline"] = -inclination + res_inc.front
#     # And now for the elevation.
#     res_elv = st_aux.elevate(-elevation)
#     if not res_elv:
#         finish = False
#         if not st_aux.elevate(-elevation + res_elv.actuator):
#             raise RuntimeError
#     instruction["elevate"] = -elevation + res_elv.actuator
#     # Generate the shift for the wheels. Note that we have to use both
#     # the wheel and wheel_aux keys, although in this special case, both have
#     # the same meaning, but this is not important in this case.
#     if re[0] is not None:
#         instruction["shift"] = \
#             st_aux.get_actuators_position(re[0]) - \
#             structure.get_actuators_position(re[0])
#     if fr[0] is not None:
#         instruction["shift_aux"] = \
#             st_aux.get_actuators_position(fr[0] + 2) - \
#             structure.get_actuators_position(fr[0] + 2)
#     # Add the end key to inform the simulator that we are done. Note that this
#     # can be False, indicating that this is not the last instruction.
#     instruction["end"] = finish
#     return instruction, st_aux
