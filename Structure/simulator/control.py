"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to generate the instructions to move the structure.

List of instructions, along with its arguments:

- advance (float): Horizontal motion. If positive, move forwards.
- elevate (float): Vertical elevation of the structure. If positive, elevate.
- incline (foat): Incline structure. If positive, the front is elevated with
    respect to the rear part of the structure.
- main: (dictionary): Main wheel (wheel closest to its corresponding step,
    that is, the first wheel that collides, or get out of the step for
    negative steps, with the stair if we moved the structure without elevating
    any wheel) with the following keys:
  - wheel (int): Index of the main wheel (0, 1, 2 or 3).
  - height (float): shift actuator. If positive, the wheel is shifted
    downwards.
  - shift (float): actual shift of the actuator once the structure has been
    elevetad/inclined.
- second (dictionary): Same as main, for the wheel in the other pair. Note
    that, at any time, only one wheel in each pair must be on the ground, and
    so, the other wheel can be on air. The main wheel is the wheel that
    mandatorily must be shifted when facing the next step, but at the same
    time, in order to gain time, we can shift another wheel in the other pair.
    This is the purpose of this key.

NOTE: height and shift are complementary data. For instance, if the structure
were elevate 10 cm, and do nothing else, all the wheel must remain in the
ground. If also "height" is -8 cm, since the wheel is initally at the ground,
after the instruction the wheel will be situated at 8 cm.
For the same case, instead of -8 cm for height, we can set shift to 2 cm,
meaning that the actuator has actually moved 2 cm from its original position to
the final one.
Height is intended for performing the actuator shift once the elevation has
been done, whereas shift in intended for performing the actuator shift at the
same time as the elevation.
"""

import copy

from physics.wheel_state import MAX_GAP


class ControlError(ValueError):
    """Error to raise when the control can not find a valid instruction"""
    pass


def last_instruction(structure, distance):
    """Generate the last instruction before finishing the program.

    Before finish the program, set the structure to its canonical position,
    that is, all the wheels in the ground, and all the actuators to their
    lower position. It returns the instruction to set the structure to that
    position.

    Arguments:
    structure -- Actual structure for which we need to compute the instruction.

    """
    # Check if all the wheels are on the ground.
    re, fr = structure.set_horizontal()
    # In case any wheel is not on the ground, also generate the instruction to
    # take the wheel down to the ground.
    # structure.DEBUG['graphics'].draw(
    #     structure.STAIRS, structure, structure.DEBUG['simulator'], False)
    # Instruction for the rear pair.
    if re[0] is not None:
        structure.shift_actuator(re[0], -re[1], check=False)
        actuator = {
            "wheel": re[0],
            "height": re[1]}
    else:
        actuator = {}
    # structure.DEBUG['graphics'].draw(
    #     structure.STAIRS, structure, structure.DEBUG['simulator'], False)
    # Same as above, for the front pair.
    # NOTE: In this case, there is no differece between the rear and front
    # pair. However, since we have to include this information in the main and
    # second actuator dictionary, we include one of then in the main and the
    # other in the second, although the other order will work as well.
    if fr[0] is not None:
        structure.shift_actuator(fr[0] + 2, -fr[1], check=False)
        act_aux = {
            "wheel": fr[0] + 2,
            "height": fr[1]}
    else:
        act_aux = {}
    # structure.DEBUG['graphics'].draw(
    #     structure.STAIRS, structure, structure.DEBUG['simulator'], False)
    structure.advance(distance, check=False)
    # Get the current inclination of the structure
    # and the current elevation.
    incline = -structure.get_inclination()
    elevate = -structure.get_elevation()
    structure.incline(incline, check=False)
    # structure.DEBUG['graphics'].draw(
    #     structure.STAIRS, structure, structure.DEBUG['simulator'], False)
    structure.elevate(elevate, check=False)
    # structure.DEBUG['graphics'].draw(
    #     structure.STAIRS, structure, structure.DEBUG['simulator'], False)
    col, stb = structure.check_position()
    if not col or not stb:
        raise ControlError
    instruction = {
        "advance": distance,
        "incline": incline,
        "elevate": elevate}
    # Although this is the last instruction, only end the execution when also
    # there is not any motion.
    if null_instruction(instruction):
        return None, None, None
    return instruction, actuator, act_aux


def make_room_wheel3(structure, height):
    """Make enough space for an actuator to complete its motion.

    When an actuator can not complete its actual motion following the
    corresponding structure motion (i.e., incline for wheel 3, elevate for the
    rest), this function computes and additional motion to make more room for
    the actuator. The function return the combination of inclination/elevation
    required for the structure.

    NOTE: This is for wheel 3. There is a corresponding function for the rest
    of the wheels.
    NOTE: Inside the function, there are lines with a call to GRAPHICS. This
    can be used to draw the structure position at any time. To do so, you need
    to include the graphics in the constructor of the structure (only for
    debug purposes).

    Arguments:
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
        raise ControlError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    if not structure.elevate(height, margin=False):
        raise ControlError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    return -res_elv.rear, height + res_elv.rear


def make_room_wheel2(structure, height):
    """See make_room_wheel3"""
    # To display the actual position of the structure, use this instruction.
    # structure.DEBUG['graphics'].draw(structure.STAIRS,
    #    structure, structure.DEBUG['simulator'], False)
    # Height is the distance we need to obtain. First, compute the total motion
    # of the actuator, which is the sum of the actual height plus the current
    # position of the actuator.
    total = structure.FRNT.REAR.d + height
    if height < 0:
        # Also, if the motion is negative (that is, the wheel must be moved
        # downwards), the total motion is the last value minus the length of
        # the actuators.
        total -= structure.HEIGHT
        # This is a auxiliary distance to prevent this actuator to cause a
        # collision while making the room. This is a value to move temporarily
        # the actuator to its lower end, ans thus, never will cause any
        # problem.
        aux_motion = structure.FRNT.REAR.d
    else:
        # This is equal as above, but when going upstairs, this error is
        # difficult to happen, and if needed, just with this value is enough.
        aux_motion = -height

    # If we try to shift the actuator, obviously we get an error,
    res_shf = structure.shift_actuator(2, -total, margin=False)
    if res_shf:
        raise ControlError
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
    # which the collision will be produced. In this case, move the actuator to
    # its lower end, and thus, this actuator will never cause any problem.
    if not structure.shift_actuator(2, -aux_motion):
        raise ControlError
    # Set the structure to the given inclination. While inclining, fix the
    # actuator, so that at the end of this function, is in the same position.
    wheel = [None, None, 0.0,  None]
    res_inc = structure.incline(inclination, elevate_rear=True, wheel=wheel)
    if not res_inc:
        # Just in case if after the inclination, the wheel collides with the
        # step, move the distance of the collision.
        structure.advance(res_inc.horizontal)
        if not structure.incline(inclination, elevate_rear=True):
            raise ControlError

    # Finally, set the actuator to its current position (remember that we
    # elevate this actuator previously).
    if not structure.shift_actuator(2, aux_motion):
        raise ControlError

    # And elevate the structure to set it back to its actual position. To get
    # the distance to elevate, first elevate a value larger than the required.
    res_elv = structure.elevate(total)
    # If this is enough, we just has finished.
    if not res_elv:
        # If not, we need to substract to the initial elevation the amount that
        # the structure collides with the second actuator.
        if not structure.elevate(total + res_elv.central):
            raise ControlError

    # And the distance is the previous value plus the distance of error given
    # in central value.
    return inclination, +total + res_elv.central - inclination


def make_room_wheel1(structure, height):
    """See make_room_wheel3."""
    # NOTE: This function is not as complete as the one for wheel2 because all
    # the errors faced in that function would not be produced for this actuator
    # in a normal structure operation.
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.REAR.FRNT.d + height
    if height < 0:
        total -= structure.HEIGHT

    res_shf = structure.shift_actuator(1, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise ControlError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(-res_shf.rear, elevate_rear=True, margin=False)
    if res_inc:
        return -res_shf.rear, res_shf.rear
    # TODO: The code here must be similar to the one in make_room_wheel2, but
    # on the other side of the structure.
    raise ControlError


def make_room_wheel0(structure, height):
    """See make_room_whell3"""

    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    total = structure.REAR.REAR.d + height
    if height < 0:
        total -= structure.REAR.REAR.LENGTH

    res_shf = structure.shift_actuator(0, -total, margin=False)
    if res_shf:
        # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
        raise ControlError
    # structure.GRAPHICS.draw(structure.STAIRS, structure, False)
    res_inc = structure.incline(-res_shf.actuator,
                                elevate_rear=True, margin=False)
    if res_inc:
        return -res_shf.actuator, res_shf.actuator

    raise ControlError


def compute_instruction(structure, wheel, hor, ver):
    """Generate the values for the next instruction.

    The function computes the horizontal motion, elevation and inclination of
    the structure to get the motion indicaded by the arguments. Note that the
    structure is modified inside this function, so, a copy of the original one
    is needed if we need to keep it.

    Parameters:
    structure -- The structure in its current position. The function need the
        structure to use its geometric function to compute distances of error.
    wheel -- Wheel we need to move and ensure enough space for it to move.
    hor, ver -- Horizontal and vertical distance that the wheel need to move to
        get to its next state.

    Return two dictionaries:
    instruction: A dictionary with the following keys (see key definition
    above):
      - advance.
      - incline.
      - elevate.
    actuator: A dictionary with the following keys (see key definition above):
      - wheel.
      - height.
    """
    instruction = {"advance": 0}
    # Simulate elevation of the actuator. Note that the vertical distance is
    # positive downwards, but the actuator position is measured in the opposite
    # direction. For that reason, we change the sign of the vertical distance.
    actuator = {
        "wheel": wheel,
        "height": -ver}
    res_shf = structure.shift_actuator(wheel, -ver, margin=False)
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
            # structure.DEBUG['graphics'].draw(
            # structure.STAIRS, structure, structure.DEBUG['simulator'],
            # pause=False)

            if not res_inc:
                # First, check i fthere is a horizontal collision.
                instruction["advance"] = res_inc.horizontal
                if not structure.advance(instruction["advance"]):
                    raise ControlError
                res_inc = structure.incline(instruction["incline"],
                                            margin=False)
                if not res_inc:
                    # Not enough space for the actuator:
                    # First step: elevate the structure the height that is
                    # actually possible.
                    instruction["incline"] += res_inc.front
                    if not structure.incline(instruction["incline"],
                                             margin=False):
                        raise ControlError
                    # And try to make more space for the actuator to complete the
                    # motion.
                    inc, elv = make_room_wheel3(structure, -res_inc.front)

                    instruction["incline"] += inc
                    instruction["elevate"] = elv
        elif wheel == 2:
            # Second actuator. I this case, the algorithm elevate the
            # structure.
            instruction["elevate"] = res_shf.central
            res_elv = structure.elevate(instruction["elevate"], margin=False)
            if not res_elv:
                # Not enough space for the actuator:
                # First step: elevate the structure the height that is
                # actually possible.
                instruction["elevate"] += res_elv.central
                if not structure.elevate(instruction["elevate"], margin=False):
                    raise ControlError
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
                    raise ControlError
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
                    raise ControlError
                # And try to make more space for the actuator to complete the
                # motion.
                inc, elv = make_room_wheel0(structure, -res_elv.central)
                instruction["incline"] = inc
                instruction["elevate"] += elv

    # Simulate the horizontal motion, to check that it is correct.
    instruction["advance"] += hor

    # Note that the the actual horizontal motion has already been performed in
    # the structure. For that reason, accumulate the horizontal motion in the
    # "advance" instruction, but only need to move this new distance.
    res_adv = structure.advance(hor)
    # structure.DEBUG['graphics'].draw(
    # structure.STAIRS, structure, structure.DEBUG['simulator'], pause=False)
    if not res_adv:
        # This error should not happen, but if so, check if it can be complete
        # correcting the error distance detected. In general, the second term
        # of the sum will be 0.
        instruction["advance"] += res_adv.horizontal
        # In this case, the hor motion has not been performed, because it
        # raises an error. For that reason, here we have to combine both
        # distances (in fact, is one of then minus the other):
        if not structure.advance(hor + res_adv.horizontal):
            raise ControlError
    # Check that the actuator can now be shifted the required height.
    if not res_shf:
        if not structure.shift_actuator(actuator["wheel"],
                                        actuator["height"],
                                        margin=False):
            raise ControlError

    # structure.DEBUG['graphics'].draw(
    # structure.STAIRS, structure, structure.DEBUG['simulator'], pause=False)

    return instruction, actuator


def compute_distance(structure, distance, next_inst=None):
    """Compute the instructions needed to cover the given distance.

    For the current state of the structure, the function computes a list of
    instruction so that the total horizontal distance covered is greater than
    the distance given.

    Returns a list of consecutive instructions.

    Arguments:
    structure
    distance
    next_inst -- If previously computed, it is possible to pass here the
      precomputed instructions, so we do not have to compute it again. In that
      case, the list of instructions returned will be equal to this list, plus
      some additional instructions if needed to cover the total distance. Also,
      the actual state of the structure is stored for each item of the list.

    """
    instructions = []
    # Compute new instructions until the total distance is covered.
    while distance > 0:
        try:
            instruction = next_inst[0]
            structure = instruction["struct"]
            next_inst = next_inst[1:]
        except (IndexError, TypeError):
            # Get next instruction.
            instruction, structure = next_instruction(structure)
        # If it is the last instruction, we can not return any more
        # instruction, and so, end here.
        if instruction is None:
            return instructions
        elif next_inst is not None:
            # Note that if we get a list of previously computed instructions,
            # it is likely that they are going to be used instead of compute
            # the same instructions again. If this is the case, they will also
            # need the actual state of the structure, and so, store it in the
            # same instruction.
            instruction["struct"] = structure
        # Take account of the distance traveled,
        distance -= instruction.get('advance', 0.0)
        # and append the instruction to the list.
        instructions.append(instruction)
    return instructions


def null_instruction(instruction):
    """Check if the instruction does nothing."""
    if abs(instruction.get("advance", 0)) > MAX_GAP:
        return False
    if abs(instruction.get("incline", 0)) > MAX_GAP:
        return False
    if abs(instruction.get("elevate", 0)) > MAX_GAP:
        return False
    main = instruction.get("main", {})
    if abs(main.get("height", 0)) > MAX_GAP:
        return False
    second = instruction.get("second", {})
    if abs(second.get("height", 0)) > MAX_GAP:
        return False
    return True


def next_instruction(structure):
    """Generate the next instruction for the structure to take the next step.

    The function gets the distances from each wheel of the struct    ure to the
    stair (which is stores in the own structure), and generate the list of
    instructions to take the next step of the stair.

    Arguments:
    structure -- Actual structure for which we need to compute the instruction.

    Return a dictionary with the instructions to perform.
    """
    # Get the distances each wheel is with respect to its closest step.
    wheel, hor, ver, w_aux, h_aux, v_aux, end \
        = structure.get_wheels_distances()

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)
    # A value equal to inf is returned when the wheel reaches the end of the
    # stair. Here, we check whether we have already reached the end of the
    # structure, and so, we have to finish the program.
    inst_aux = {}
    act_aux = {}
    if end:
        # Computing the las instruction before finishing the program.
        instruction, actuator, act_aux = last_instruction(st_aux, hor)
        if instruction is None:
            return None, None
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
        k = structure.WIDTH / 1000
        v_total = v_aux * (hor + k) / (h_aux + k)
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
    # Check if the control has generated an instruction that does nothing. If
    # we do not control this error, the program get hung because the strcture
    # does not move but the program does not finishes.
    if null_instruction(instruction):
        raise ControlError

    return instruction, st_aux


def manual_control(key_pressed, simulator):
    """Function to convert a key to a instruction."""
    if key_pressed == ord('4'):
        command = {'advance': -simulator.profile.speed}
    elif key_pressed == ord('6'):
        command = {'advance': +simulator.profile.speed}
    elif key_pressed == ord('2'):
        command = {'elevate': -simulator.speed_elevate_dw}
    elif key_pressed == ord('8'):
        command = {'elevate': +simulator.speed_elevate_up}
    elif key_pressed == ord('5'):
        command = {'reset': True}
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
        command = None
    return command

###############################################################################
# End of file.
###############################################################################