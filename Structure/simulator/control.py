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

# structure.DEBUG['graphics'].draw(structure.STAIRS, structure, structure.DEBUG['simulator'], False)


import copy

from physics.wheel_state import MAX_GAP


class ControlError(ValueError):
    """Error to raise when the control can not find a valid instruction"""
    pass


class FinalInstruction(Exception):
    """Exception to set the final instruction.

    Final instruction can be due to reaching the end of the stair, or because
    the control module can not find an instruction to continue.

    """
    pass


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
    # second = instruction.get("second", {})
    # if abs(second.get("height", 0)) > MAX_GAP:
    #     return False
    return True


def last_instruction(structure):
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
    # Instruction for the rear pair.
    if re[0] is not None:
        if not structure.push_actuator(re[0], -re[1]):
            # Note that, when pusing the actuator, the structure can be
            # inclined, and also moved horizontally.
            raise RuntimeError
        actuator = {
            "wheel": re[0],
            "height": re[1]}
    else:
        actuator = {}
    # Same as above, for the front pair.
    # NOTE: In this case, there is no differece between the rear and front
    # pair. However, since we have to include this information in the main and
    # second actuator dictionary, we include one of then in the main and the
    # other in the second, although the other order will work as well.
    if fr[0] is not None:
        if not structure.push_actuator(fr[0] + 2, -fr[1]):
            raise RuntimeError
        act_aux = {
            "wheel": fr[0] + 2,
            "height": fr[1]}
    else:
        act_aux = {}
    # Get the current inclination of the structure and the current elevation,
    # to set the structure to the initial position.
    incline = -structure.get_inclination()
    elevate = -structure.get_elevation()
    if not structure.incline(incline):
        raise RuntimeError
    if not structure.elevate(elevate):
        raise RuntimeError
    # Return the actuator shift.
    # To get the structure motion, the calling function only need to check the
    # motion beween the original structure, and the structure given to this
    # function.
    return actuator, act_aux


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
    wheel, hor, ver, w_aux, h_aux, v_aux, end = \
        structure.get_wheels_distances()

    # Create a deep copy of the structure, to simulate all the motions computed
    # without modifying the actual structure.
    st_aux = copy.deepcopy(structure)

    # Perform the horizontal motion.
    # NOTE that, when pushing the actuator, the structure can be moved more if
    # there were some collision when inclining.
    res_adv = st_aux.advance(hor)
    if not res_adv:
        hor += res_adv.horizontal()
        if not st_aux.advance(hor):
            raise RuntimeError

    # The variable end tells if we are in the last instruction of the motion.
    if end:
        # Computing the las instruction before finishing the program.
        actuator, act_aux = last_instruction(st_aux)
        wheel = actuator.get("wheel", None)
        w_aux = act_aux.get("wheel", None)
    else:
        # Get the next instruction for the main wheel.
        state = st_aux.push_actuator(wheel, -ver)
        # Set the values to the shift for the actuators.
        actuator = {
            "wheel": wheel,
            "height": ver}
        if not state:
            actuator["height"] -= state.elevation()
        # And the next instruction for the wheel in the other pair.
        # NOTE: The height to shift the second actuator must be proportional
        # to the horizontal distance this wheel must move compared with the
        # horizontal distance for the main wheel. This ensure that the motion
        # for the second actuator is more regular.
        # To prevent for a zero division, add a small amount to both horizontal
        # distances. To make it invariant to system scale, add a value
        # proportional to the size of the structure.
        k = structure.position.WIDTH / 1000
        v_total = v_aux * (hor + k) / (h_aux + k)
        # Now, we check if we can shift the actuator the distance required.
        res_act = st_aux.shift_actuator(w_aux, -v_total)
        if not res_act:
            v_total -= res_act.actuator(w_aux)
            if not st_aux.shift_actuator(w_aux, -v_total):
                raise ControlError
        act_aux = {
            "wheel": w_aux,
            "height": -v_total}

    # Get the motion done by the structure to shift the actuator.
    motion = st_aux.get_motion(structure)

    instruction = {
        "advance": motion.get_horizontal(),
        "incline": motion.get_inclination(),
        "elevate": motion.get_vertical()}

    # Get the shift of each actuator. This value is needed in case we have to
    # return the actual shift of the actuator, not the shift after the
    # elevation/inclination.
    # This value con be computed from the diference in shift for the actuator
    # at the current position minus the same shift in the initial position.
    try:
        actuator["shift"] = \
            st_aux.get_actuator_position(wheel) - \
            structure.get_actuator_position(wheel)
        instruction["main"] = actuator
    except ValueError:
        pass
    try:
        act_aux["shift"] = st_aux.get_actuator_position(w_aux) - \
            structure.get_actuator_position(w_aux)
        instruction["second"] = act_aux
    except ValueError:
        pass
    # Check if the control has generated an instruction that does nothing. If
    # we do not control this error, the program get hung because the strcture
    # does not move but the program does not finishes.
    if null_instruction(instruction):
        return None, st_aux

    return instruction, st_aux


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


###############################################################################

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
        command = {'incline': +simulator.speed_incline_up, 'fixed': 0}
    elif key_pressed == ord('g'):
        command = {'incline': -simulator.speed_incline_dw, 'fixed': 0}
    elif key_pressed == ord('y'):
        command = {'incline': +simulator.speed_incline_up, 'fixed': 1}
    elif key_pressed == ord('h'):
        command = {'incline': -simulator.speed_incline_dw, 'fixed': 1}
    elif key_pressed == ord('u'):
        command = {'incline': +simulator.speed_incline_up, 'fixed': 2}
    elif key_pressed == ord('j'):
        command = {'incline': -simulator.speed_incline_dw, 'fixed': 2}
    elif key_pressed == ord('i'):
        command = {'incline': +simulator.speed_incline_dw, 'fixed': 3}
    elif key_pressed == ord('k'):
        command = {'incline': -simulator.speed_incline_up, 'fixed': 3}
    ###########################################################################
    else:
        command = None
    return command

###############################################################################
# End of file.
###############################################################################

# def compute_instruction(structure, wheel, hor, ver):
#     """Generate the values for the next instruction.
#
#     The function computes the horizontal motion, elevation and inclination of
#     the structure to get the motion indicaded by the arguments. Note that the
#     structure is modified inside this function, so, a copy of the original one
#     is needed if we need to keep it.
#
#     Parameters:
#     structure -- The structure in its current position. The function needs the
#         structure to use its geometric function to compute distances of error.
#     wheel -- Wheel we need to move and ensure enough space for it to move.
#     hor, ver -- Horizontal and vertical distance that the wheel need to move to
#         get to its next state.
#
#     Return two dictionaries:
#     instruction: A dictionary with the following keys (see key definition
#     above):
#       - advance.
#       - incline.
#       - elevate.
#     actuator: A dictionary with the following keys (see key definition above):
#       - wheel.
#       - height.
#     """
#     instruction = {
#         "advance": hor,
#         "incline": 0.0,
#         "elevate": 0.0}
#
#     res_adv = structure.advance(hor)
#     if not res_adv:
#         raise ControlError
#     # Simulate elevation of the actuator. Note that the vertical distance is
#     # positive downwards, but the actuator position is measured in the opposite
#     # direction. For that reason, we change the sign of the vertical distance.
#     actuator = {
#         "wheel": wheel,
#         "height": -ver}
#     res_shf = structure.shift_actuator(wheel, -ver, margin=False)
#     if not res_shf:
#         act = res_shf.actuator
#         if not structure.shift_actuator(wheel, -ver + act, margin=False):
#             raise ControlError
#         ver = act
#
#         # If the actuator can not be sifted, we have to make room for the
#         # actuator to compete the motion. This action depends on the index
#         # of the actuator. The height to consider is given by parameter
#         # "actuator". Continue reading the code:
#         #######################################################################
#         height = res_shf.actuator
#         # Height is the space we have to make in the structure to allow
#         # the required motion for the actuator.
#         if wheel == 3:
#             inc, elv, adv = make_room_wheel3(structure, height)
#         elif wheel == 2:
#             inc, elv, adv = make_room_wheel2(structure, height)
#         elif wheel == 1:
#             inc, elv, adv = make_room_wheel1(structure, height)
#         elif wheel == 0:
#             inc, elv, adv = make_room_wheel0(structure, height)
#
#         instruction["incline"] += inc
#         instruction["elevate"] += elv
#         instruction["advance"] += adv
#
#     # Check that the actuator can now be shifted the required height.
#     if not res_shf:
#         res_shf = structure.shift_actuator(wheel, -ver, margin=False)
#         if not res_shf:
#             raise ControlError
#
#     return instruction, actuator
#
#
# def next_instruction(structure):
#     """Generate the next instruction for the structure to take the next step.
#
#     The function gets the distances from each wheel of the struct    ure to the
#     stair (which is stores in the own structure), and generate the list of
#     instructions to take the next step of the stair.
#
#     Arguments:
#     structure -- Actual structure for which we need to compute the instruction.
#
#     Return a dictionary with the instructions to perform.
#     """
#     # Get the distances each wheel is with respect to its closest step.
#     wheel, hor, ver, w_aux, h_aux, v_aux, end \
#         = structure.get_wheels_distances()
#
#     # Create a deep copy of the structure, to simulate all the motions computed
#     # without modifying the actual structure.
#     st_aux = copy.deepcopy(structure)
#     # A value equal to inf is returned when the wheel reaches the end of the
#     # stair. Here, we check whether we have already reached the end of the
#     # structure, and so, we have to finish the program.
#     if end:
#         # Computing the las instruction before finishing the program.
#         instruction, actuator, act_aux = last_instruction(st_aux, hor)
#         if instruction is None:
#             return None, None
#         wheel = actuator.get("wheel", 0)
#         w_aux = act_aux.get("wheel", 2)
#     else:
#         # Get the next instruction for the main wheel.
#         instruction, actuator = compute_instruction(st_aux, wheel, hor, ver)
#         # And the next instruction for the wheel in the other pair.
#         # NOTE: The height to shift the second actuator must be proportional
#         # to the horizontal distance this wheel must move compared with the
#         # horizontal distance for the main wheel. This ensure that the motion
#         # for the second actuator is more regular.
#         # To prevent for a zero division, add a small amount to both horizontal
#         # distances. To make it invariant to system scale, add a value
#         # proportional to the size of the structure.
#         k = structure.WIDTH / 1000
#         v_total = v_aux * (hor + k) / (h_aux + k)
#         # Now, we check if we can shift the actuator the distance requirede.
#         res_act = st_aux.shift_actuator(w_aux, -v_total)
#         if not res_act:
#             v_total -= res_act.actuator
#             if not st_aux.shift_actuator(w_aux, -v_total):
#                 raise ControlError
#         act_aux = {
#             "wheel": w_aux,
#             "height": -v_total}
#
#     # Get the shift of each actuator. This value is needed in case we have to
#     # return the actual shift of the actuator, not the shift after the
#     # elevation/inclination.
#     # This value con be computed from the diference in shift for the actuator
#     # at the current position minus the same shift in the initial position.
#     actuator["shift"] = \
#         st_aux.get_actuator_position(wheel) - \
#         structure.get_actuator_position(wheel)
#     instruction["main"] = actuator
#     act_aux["shift"] = st_aux.get_actuator_position(
#         w_aux) - structure.get_actuator_position(w_aux)
#     instruction["second"] = act_aux
#     # Check if the control has generated an instruction that does nothing. If
#     # we do not control this error, the program get hung because the strcture
#     # does not move but the program does not finishes.
#     if null_instruction(instruction):
#         raise ControlError
#
#     return instruction, st_aux
#


# def make_room_wheel3(structure, height):
#     """Make enough space for an actuator to complete its motion.
#
#     When the actuator have not enough space to complete the required motion,
#     we need to make room for this motion. In the case of the front wheel (4),
#     the main option is incline from the front. This can be enough for most of
#     the cases, but in some cases, the central actuators can collide with the
#     structure. In this case, we need to elevate the structure in the opposite
#     direction to allow the structure to incline the required height.
#
#     NOTE: This is for wheel 3. There is a corresponding function for the rest
#     of the wheels.
#     NOTE: Inside the function, there are lines with a call to GRAPHICS. This
#     can be used to draw the structure position at any time. To do so, you need
#     to include the graphics in the constructor of the structure (only for
#     debug purposes).
#
#     Arguments:
#     height -- additional height the actuator must be shifted and can not be
#         completed with a single motion.
#
#     """
#     # For wheel 3, the first kind of motion is incline fron the front.
#
#     # Try to incline the structure to get the space required.
#     res_inc = structure.incline(height, margin=False)
#     if res_inc:
#         return height, 0.0, 0.0
#     # If the motion failed, check if it is just because when inclining one of
#     # the wheels has move inside a step, that is, check if there is a
#     # horizontal collision.
#     total_advance = res_inc.horizontal
#     if not structure.advance(total_advance):
#         raise ControlError
#     # Try again to incline the structure to get the space required.
#     res_inc = structure.incline(height, margin=False)
#     if res_inc:
#         return height, 0.0, total_advance
#     # In this case, it could be two possible reasons:
#     # - The structure is colliding with the second or third actuator.
#     # - The height of the step is greater than the length of the actuator. In
#     #   this case, there is not possiblity to climb the stair.
#     # Check if it is the first case:
#
#     total_elevate = -res_inc.front
#     res_elv = structure.elevate(total_elevate, margin=False)
#     if not res_elv:
#         total_elevate += res_elv.central
#         if not structure.elevate(total_elevate, margin=False):
#             raise ControlError
#
#     total_incline = height - total_elevate
#     res_inc = structure.incline(total_incline, margin=False)
#     if res_inc:
#         return total_incline, total_elevate, total_advance
#     # Check again if the problem is a horizontal collision.
#     new_advance = res_inc.horizontal
#     if not structure.advance(new_advance):
#         raise ControlError
#     total_advance += new_advance
#     # Try again to incline the structure to get the space required.
#     res_inc = structure.incline(total_incline, margin=False)
#     if res_inc:
#         return total_incline, total_elevate, total_advance
#
#     # In this case, it is possible that the structure can perform the motion,
#     # but an actuator does not let the structure move. In this case, we can
#     # try to do the same action, but withut checking errors when elevating.
#     # Since no errors are checked, the inclination must be controlled by a
#     # try block, otherwise an error can be raised.
#     new_elevate = +res_inc.rear
#     total_elevate += new_elevate
#     structure.elevate(new_elevate, check=False)
#     total_incline -= res_inc.rear
#     try:
#         res_inc = structure.incline(total_incline, margin=False)
#     except RuntimeError:
#         raise ControlError
#     if res_inc:
#         return total_incline, total_elevate, total_advance
#
#     # Check again if the problem is a horizontal collision.
#     new_advance = res_inc.horizontal
#     if not structure.advance(new_advance):
#         raise ControlError
#     total_advance += new_advance
#     # Try again to incline the structure to get the space required.
#     try:
#         res_inc = structure.incline(total_incline, margin=False)
#     except RuntimeError:
#         raise ControlError
#     if res_inc:
#         return total_incline, total_elevate, total_advance
#
#     raise ControlError
#
#
# def make_room_wheel2(structure, height):
#
#     # Try to incline the structure to get the space required.
#     res_elv = structure.elevate(height, margin=False)
#     if res_elv:
#         return 0.0, height, 0.0
#     # In this case, the problem is a collision with the second actuator.
#     # Elevate the structure the distance that is actually possible.
#     total_elevate = height + res_elv.central
#     if not structure.elevate(total_elevate, margin=False):
#         raise ControlError
#     height = -res_elv.central
#
#     # The final inclination can be computed with this auxiliary function of
#     # the structure.
#     total_incline = structure.get_inclination_central_wheels(0, height)
#     # If we try to incline this value, we get the error.
#     res_inc = structure.incline(total_incline, margin=False)
#     if res_inc:
#         raise ControlError
#     # And the actuator error will tell us the amount of elevation.
#     new_elevate = res_inc.actuator
#     total_elevate += new_elevate
#     # Elevate the structure, but without checking errors.
#     structure.elevate(new_elevate, check=False)
#     try:
#         # And now incline again, but checking for errors.
#         res_inc = structure.incline(total_incline, margin=False)
#     except RuntimeError:
#         raise ControlError
#
#     if res_inc:
#         return total_incline, total_elevate, 0.0
#
#     # Check again if the problem is a horizontal collision.
#     total_advance = res_inc.horizontal
#     if not structure.advance(total_advance):
#         raise ControlError
#     # Try again to incline the structure to get the space required.
#     try:
#         res_inc = structure.incline(total_incline, margin=False)
#     except RuntimeError:
#         raise ControlError
#     if res_inc:
#         return total_incline, total_elevate, total_advance
#
#     raise ControlError
#
#
# """
# st = structure
# st.DEBUG['graphics'].draw(st.STAIRS, st, st.DEBUG['simulator'], False)
# """
#
#
# def make_room_wheel1(structure, height):
#
#     # Try to elevate the structure to get the space required.
#     res_elv = structure.elevate(height, margin=False)
#     if res_elv:
#         return 0.0, height, 0.0
#     # If not possible, elevate just the height allowed by the structure.
#     total_elevate = height + res_elv.central
#     if not structure.elevate(total_elevate, margin=False):
#         raise ControlError
#
#     # And get the rest of the space inclining from the rear side.
#     res_act = structure.shift_actuator(1, -height)
#     if res_act:
#         raise ControlError
#     total_incline = -res_act.rear
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         # Note that, when elevating the rear wheel, the inclination is done
#         # elevating from the rear, but the simulator does not have this option,
#         # only can elevate from the front. To correct this, when returning the
#         # correct elevation, we have to substract the total inclination to
#         # make an equivalent motion.
#         return total_incline, total_elevate - total_incline, 0.0
#     # Check if the collision is due to a horizontal displacement.
#     total_advance = res_inc.horizontal
#     if not structure.advance(total_advance):
#         raise ControlError
#     # Try again to check if it was just a horizontal collision.
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#
#     # If we reach this line, the problem is a collision with the thrid
#     # actuator. In this case, the final inclination can be computed with this
#     # auxiliary function of t    he structure.
#     height = -res_elv.central
#
#     total_incline = structure.get_inclination_central_wheels(height, 0)
#     # If we try to incline this value, we get the error.
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#     # And the actuator error will tell us the amount of elevation.
#     new_elevate = res_inc.actuator
#     total_elevate += new_elevate
#     # Elevate the structure, but without checking errors.
#     structure.elevate(new_elevate, check=False)
#     try:
#         # And now incline again, but checking for errors.
#         res_inc = structure.incline(
#             total_incline, elevate_rear=True, margin=False)
#     except RuntimeError:
#         raise ControlError
#
#     if res_inc:
#         return total_incline, total_elevate - total_incline, 0.0
#
#     # Check again if the problem is a horizontal collision.
#     new_advance = res_inc.horizontal
#     total_advance += new_advance
#     if not structure.advance(new_advance):
#         raise ControlError
#     # Try again to incline the structure to get the space required.
#     try:
#         res_inc = structure.incline(
#             total_incline, elevate_rear=True, margin=False)
#     except RuntimeError:
#         raise ControlError
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#
#     raise ControlError
#
#
# def make_room_wheel0(structure, height):
#
#     # Try to elevate the structure to get the space required.
#     res_elv = structure.elevate(height, margin=False)
#     if res_elv:
#         return 0.0, height, 0.0
#     # If not possible, elevate just the height allowed by the structure.
#     total_elevate = height + res_elv.central
#     if not structure.elevate(total_elevate, margin=False):
#         raise ControlError
#
#     # And get the rest of the space inclining from the rear side.
#     total_incline = res_elv.central
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         # Note that, when elevating the rear wheel, the inclination is done
#         # elevating from the rear, but the simulator does not have this option,
#         # only can elevate from the front. To correct this, when returning the
#         # correct elevation, we have to substract the total inclination to
#         # make an equivalent motion.
#         return total_incline, total_elevate - total_incline, 0.0
#     # Check if the collision is due to a horizontal displacement.
#     total_advance = res_inc.horizontal
#     if not structure.advance(total_advance):
#         raise ControlError
#     # Try again to check if it was just a horizontal collision.
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#     # If we reach this line, the problem is a collision with the thrid
#     # actuator. We have to elevate the distance given in the parameter front
#     elevate_front = res_inc.front
#     if not structure.elevate(elevate_front, margin=False):
#         raise ControlError
#     total_elevate += elevate_front
#     # And incline the height needed, plus the distance elevated (in general,
#     # each distance will be of opposite signs).
#     total_incline += elevate_front
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#     # Check again if the collision is due to a horizontal displacement.
#     advance_front = res_inc.horizontal
#     if not structure.advance(advance_front):
#         raise ControlError
#     total_advance += advance_front
#     # And try again.
#     res_inc = structure.incline(total_incline, elevate_rear=True, margin=False)
#     if res_inc:
#         return total_incline, total_elevate - total_incline, total_advance
#
#     raise ControlError

###############################################################################
# def make_room_wheel3(structure, height):
#     """Make enough space for an actuator to complete its motion.
#
#     When an actuator can not complete its actual motion following the
#     corresponding structure motion (i.e., incline for wheel 3, elevate for the
#     rest), this function computes and additional motion to make more room for
#     the actuator. The function return the combination of inclination/elevation
#     required for the structure.
#
#     NOTE: This is for wheel 3. There is a corresponding function for the rest
#     of the wheels.
#     NOTE: Inside the function, there are lines with a call to GRAPHICS. This
#     can be used to draw the structure position at any time. To do so, you need
#     to include the graphics in the constructor of the structure (only for
#     debug purposes).
#
#     Arguments:
#     height -- additional height the actuator must be shifted and can not be
#         completed with a single motion.
#
#     """
#     # For wheel 3, if the motion can not be completed, can only be due to the
#     # second actuator. Try to elevate the structure.
#     res_elv = structure.elevate(height, margin=False)
#     if res_elv:
#         return 0.0, height, 0.0
#     # In this case, the parameter rear indicates the height to incline the
#     # structure from the rear to allow the inclination to complete.
#     # To make the space necessary, we have to simultaneously incline and
#     # elevate in opossite directions. To get that, first make both motions
#     # without checking any collisions, and then check for collisions.
#     structure.incline(-res_elv.rear, elevate_rear=True,
#                       check=False, margin=False)
#     # raise ControlError
#     structure.elevate(height, check=False, margin=False)
#     # Now check any collision. This can be due to two reasons:
#     # It is impossible to make more room with a combination of both motions.
#     # This instruction is impossible, and so, finish the program.
#     # When inclining, the front wheel has collided with the step. In this case,
#     # move the structure in the opossite direction.
#     res_col, __ = structure.check_position()
#     if not res_col:
#         if not structure.advance(res_col.horizontal):
#             raise ControlError
#     return -res_elv.rear, height + res_elv.rear, res_col.horizontal
#
#
# def make_room_wheel2(structure, height):
#     """See make_room_wheel3"""
#     # To display the actual position of the structure, use this instruction.
#     # Height is the distance we need to obtain. First, compute the total motion
#     # of the actuator, which is the sum of the actual height plus the current
#     # position of the actuator.
#     total = structure.FRNT.REAR.d + height
#     if height < 0:
#         # Also, if the motion is negative (that is, the wheel must be moved
#         # downwards), the total motion is the last value minus the length of
#         # the actuators.
#         total -= structure.HEIGHT
#         # This is a auxiliary distance to prevent this actuator to cause a
#         # collision while making the room. This is a value to move temporarily
#         # the actuator to its lower end, ans thus, never will cause any
#         # problem.
#         aux_motion = structure.FRNT.REAR.d
#     else:
#         # This is equal as above, but when going upstairs, this error is
#         # difficult to happen, and if needed, just with this value is enough.
#         aux_motion = -height
#
#     # If we try to shift the actuator, obviously we get an error,
#     res_shf = structure.shift_actuator(2, -total, margin=False)
#     if res_shf:
#         raise ControlError
#     # but the value front indicates the inclination for the structure to get
#     # the space requirede.
#     res_inc = structure.incline(res_shf.front, margin=False)
#     if res_inc:
#         return res_shf.front, 0
#
#     # If the code gets here, is because the collision is with the second
#     # actuator. In this case, compute the final inclination of the structure
#     # after the third actuator has been shifted.
#     inclination = structure.get_inclination_central_wheels(0, height)
#     # Shift temporarily the third actuator to prevent collision with this
#     # actuator, instead that with the second, which is the actual one with
#     # which the collision will be produced. In this case, move the actuator to
#     # its lower end, and thus, this actuator will never cause any problem.
#     if not structure.shift_actuator(2, -aux_motion):
#         raise ControlError
#     # Set the structure to the given inclination. While inclining, fix the
#     # actuator, so that at the end of this function, is in the same position.
#     wheel = [None, None, 0.0,  None]
#     res_inc = structure.incline(inclination, elevate_rear=True, wheel=wheel)
#     if not res_inc:
#         # Just in case if after the inclination, the wheel collides with the
#         # step, move the distance of the collision.
#         structure.advance(res_inc.horizontal)
#         if not structure.incline(inclination, elevate_rear=True):
#             raise ControlError
#
#     # Finally, set the actuator to its current position (remember that we
#     # elevate this actuator previously).
#     if not structure.shift_actuator(2, aux_motion):
#         raise ControlError
#
#     # And elevate the structure to set it back to its actual position. To get
#     # the distance to elevate, first elevate a value larger than the required.
#     res_elv = structure.elevate(total)
#     # If this is enough, we just has finished.
#     if not res_elv:
#         # If not, we need to substract to the initial elevation the amount that
#         # the structure collides with the second actuator.
#         if not structure.elevate(total + res_elv.central):
#             raise ControlError
#
#     # And the distance is the previous value plus the distance of error given
#     # in central value.
#     return inclination, +total + res_elv.central - inclination
#
#
# def make_room_wheel1(structure, height):
#     """See make_room_wheel3."""
#     # NOTE: This function is not as complete as the one for wheel2 because all
#     # the errors faced in that function would not be produced for this actuator
#     # in a normal structure operation.
#     total = structure.REAR.FRNT.d + height
#     if height < 0:
#         total -= structure.HEIGHT
#
#     res_shf = structure.shift_actuator(1, -total, margin=False)
#     if res_shf:
#         raise ControlError
#     res_inc = structure.incline(-res_shf.rear, elevate_rear=True, margin=False)
#     if res_inc:
#         return -res_shf.rear, res_shf.rear
#     # TODO: The code here must be similar to the one in make_room_wheel2, but
#     # on the other side of the structure.
#     raise ControlError
#
#
# def make_room_wheel0(structure, height):
#     """See make_room_whell3"""
#
#     total = structure.REAR.REAR.d + height
#     if height < 0:
#         total -= structure.REAR.REAR.LENGTH
#
#     res_shf = structure.shift_actuator(0, -total, margin=False)
#     if res_shf:
#         raise ControlError
#     res_inc = structure.incline(-res_shf.actuator,
#                                 elevate_rear=True, margin=False)
#     if res_inc:
#         return -res_shf.actuator, res_shf.actuator
#
#     raise ControlError
#
#
# def compute_instruction(structure, wheel, hor, ver):
#     """Generate the values for the next instruction.
#
#     The function computes the horizontal motion, elevation and inclination of
#     the structure to get the motion indicaded by the arguments. Note that the
#     structure is modified inside this function, so, a copy of the original one
#     is needed if we need to keep it.
#
#     Parameters:
#     structure -- The structure in its current position. The function need the
#         structure to use its geometric function to compute distances of error.
#     wheel -- Wheel we need to move and ensure enough space for it to move.
#     hor, ver -- Horizontal and vertical distance that the wheel need to move to
#         get to its next state.
#
#     Return two dictionaries:
#     instruction: A dictionary with the following keys (see key definition
#     above):
#       - advance.
#       - incline.
#       - elevate.
#     actuator: A dictionary with the following keys (see key definition above):
#       - wheel.
#       - height.
#     """
#     instruction = {"advance": 0}
#     # Simulate elevation of the actuator. Note that the vertical distance is
#     # positive downwards, but the actuator position is measured in the opposite
#     # direction. For that reason, we change the sign of the vertical distance.
#     actuator = {
#         "wheel": wheel,
#         "height": -ver}
#     res_shf = structure.shift_actuator(wheel, -ver, margin=False)
#     if not res_shf:
#         # If the actuator can not be sifted, we have to make room for the
#         # actuator to compete the motion. This action depends on the index
#         # of the actuator. The height to consider is given by parameter
#         # "central". Continue reading the code:
#         #######################################################################
#         if wheel == 3:
#             # Front actuator. In this case, the algorithm incline the
#             # structure.
#             instruction['incline'] = res_shf.actuator
#             res_inc = structure.incline(instruction["incline"], margin=False)
#
#             if not res_inc:
#                 # First, check if there is a horizontal collision.
#                 instruction["advance"] = res_inc.horizontal
#                 if not structure.advance(instruction["advance"]):
#                     raise ControlError
#                 # Try to incline again, to see if the problem is that, when
#                 # inclining, there is a horizontal collision with one wheel.
#                 res_inc = structure.incline(instruction["incline"],
#                                             margin=False)
#             if not res_inc:
#                 # Not enough space for the actuator:
#                 # First step: incline the structure the height that is
#                 # actually possible.
#                 instruction["incline"] += res_inc.front
#                 if not structure.incline(instruction["incline"], margin=False):
#                     # Here, the problem is that the structure can not incline
#                     # more. Return just the inclination the structure can
#                     # perform, although in the next instruction the structure
#                     # can not move anymore, and finish the program.
#                     instruction["incline"] += res_inc.central
#                     if not structure.incline(instruction["incline"],
#                                              margin=False):
#                         raise ControlError
#                 else:
#                     # And try to make more space for the actuator to complete
#                     # the motion.
#                     inc, elv, adv = make_room_wheel3(
#                         structure, -res_inc.central)
#
#                     instruction["incline"] += inc
#                     instruction["elevate"] = elv
#                     instruction["advance"] += adv
#         elif wheel == 2:
#             # Second actuator. I this case, the algorithm elevate the
#             # structure.
#             instruction["elevate"] = res_shf.central
#             res_elv = structure.elevate(instruction["elevate"], margin=False)
#             if not res_elv:
#                 # Not enough space for the actuator:
#                 # First step: elevate the structure the height that is
#                 # actually possible.
#                 instruction["elevate"] += res_elv.central
#                 if not structure.elevate(instruction["elevate"], margin=False):
#                     raise ControlError
#                 # And try to make more space for the actuator to complete the
#                 # motion.
#                 inc, elv = make_room_wheel2(structure, -res_elv.central)
#                 instruction["incline"] = inc
#                 instruction["elevate"] += elv
#         elif wheel == 1:
#             # Same as wheel 2.
#             # Second actuator. I this case, the algorithm elevate the
#             # structure. In this case, the algorithm elevate the structure.
#             instruction["elevate"] = res_shf.central
#             res_elv = structure.elevate(instruction["elevate"], margin=False)
#             if not res_elv:
#                 # Not enough space for the actuator:
#                 # First step: elevate the structure the height that is
#                 # actually possible.
#                 instruction["elevate"] += res_elv.central
#                 if not structure.elevate(instruction["elevate"], margin=False):
#                     raise ControlError
#                 # And try to make more space for the actuator to complete the
#                 # motion.
#                 inc, elv = make_room_wheel1(structure, -res_elv.central)
#                 instruction["incline"] = inc
#                 instruction["elevate"] += elv
#         elif wheel == 0:
#             # Same as wheel 1.
#             # Second actuator. I this case, the algorithm elevate the
#             # structure. In this case, the algorithm elevate the structure.
#             instruction["elevate"] = res_shf.central
#             res_elv = structure.elevate(instruction["elevate"], margin=False)
#             if not res_elv:
#                 # Not enough space for the actuator:
#                 # First step: elevate the structure the height that is
#                 # actually possible.
#                 instruction["elevate"] += res_elv.central
#                 if not structure.elevate(instruction["elevate"], margin=False):
#                     raise ControlError
#                 # And try to make more space for the actuator to complete the
#                 # motion.
#                 inc, elv = make_room_wheel0(
#                     structure, instruction['advance'] - res_elv.central)
#                 instruction["incline"] = inc
#                 instruction["elevate"] += elv
#
#     # Simulate the horizontal motion, to check that it is correct.
#     # Note that the previous code could have moved the structure due to
#     # collisions when inclining the structure. However, we add the original
#     # motion to the total motion, and check if there is any other collision.
#     instruction["advance"] += hor
#
#     # Note that the the actual horizontal motion has already been performed in
#     # the structure. For that reason, accumulate the horizontal motion in the
#     # "advance" instruction, but only need to move this new distance.
#     res_adv = structure.advance(hor)
#     if not res_adv:
#         # This error should not happen, but if so, check if it can be complete
#         # correcting the error distance detected. In general, the second term
#         # of the sum will be 0.
#         instruction["advance"] += res_adv.horizontal
#         # In this case, the hor motion has not been performed, because it
#         # raises an error. For that reason, here we have to combine both
#         # distances (in fact, is one of then minus the other):
#         if not structure.advance(hor + res_adv.horizontal):
#             raise ControlError
#     # Check that the actuator can now be shifted the required height.
#     if not res_shf:
#         res_shf = structure.shift_actuator(actuator["wheel"],
#                                            actuator["height"],
#                                            margin=False)
#         if not res_shf:
#             actuator["height"] += res_shf.actuator
#         if not structure.shift_actuator(actuator["wheel"],
#                                         actuator["height"],
#                                         margin=False):
#             raise ControlError
#
#     return instruction, actuator
