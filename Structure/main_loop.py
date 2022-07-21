"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

"""

import sys

import readXML
from physics import stairs
from structure import base
from simulator.simulator import Simulator, SimulatorState
from graphics.graphics import Graphics
from simulator import control


# Open and check settings file.
try:
    settings_name = sys.argv[1]
except Exception:
    settings_name = "settings.xml"

# Read stairs data and create physical stairs object.
stairs_list, landing = readXML.read_stairs(settings_name)
stairs = stairs.Stair(stairs_list, landing)
# Read structure dimensions and create structure.
__, structure_size, wheels_radius = readXML.read_structure(settings_name)

# Read simulator data.
dynamics_data, sample_data = readXML.read_dynamics(settings_name)
sm = Simulator(dynamics_data, sample_data)
res = SimulatorState.SimulatorOK

# Read graphical variables.
image_data, video_data, csv_data = readXML.read_graphics(settings_name)
axis = {
    "height": structure_size["d"] + video_data['margin'],
    "max_speed": 1.2 * dynamics_data["speed"],
    "max_incline": structure_size['n'] + video_data['margin']}
graphics = Graphics(image_data, video_data, csv_data, sample_data, axis)

debug = {'graphics': graphics, 'simulator': sm}
# debug = None
structure = base.Base(structure_size, wheels_radius, stairs)  # , debug=debug)

# instruction = {
#     'wheel': 3, 'height': -120, 'advance': 300}
# sm.simulate_instruction(structure, instruction)
# instruction = {'advance': 100}
# sm.simulate_instruction(structure, instruction)
# instruction = {'incline': -90, 'fixed': 3}
# sm.simulate_instruction(structure, instruction)
# instruction = {'wheel': 2, 'height': -50}
# sm.simulate_instruction(structure, instruction)
# instruction = {
#     'wheel': 1, 'height': -50}
# sm.simulate_instruction(structure, instruction)
# instruction = {
#     'wheel': 2, 'height': -150}
# sm.simulate_instruction(structure, instruction)

# instruction = {
#     'wheel': 2, 'height': -10}
# sm.simulate_instruction(structure, instruction)


# Draw initial state of the structure.
continue_loop, key_pressed = graphics.draw(stairs, structure, sm.counter)
# Continue_loop is a flag to help finish the program. It gets False value when
# the user press the Esc key (see graphics module).
# Main loop
instruction_number = 0

while continue_loop:
    if graphics.manual_mode:
        # In manual mode, wait for the user to press a instruction.
        continue_loop, key_pressed = graphics.draw(
            stairs, structure, sm.counter)
        instruction = control.manual_control(key_pressed, sm)
        str_aux = structure
        sm.simulate_instruction(structure, instruction)
    else:
        # Compute instruction:
        # Compute the next instruction.
        instruction, str_aux = control.next_instruction(structure)
        if instruction is None:
            try:
                graphics.set_manual_mode()
            except ValueError:
                # If the graphics can not be set in manual mode (normally
                # because we are not displaying images), finish the loop.
                continue_loop = False
            continue
        stop_lentgh = sm.stop_distance(instruction)
        next_instructions = control.compute_distance(str_aux, stop_lentgh)
        # Compute and initial estimation of the time requirede to
        # complete the instruction, and if dynamics is implemented,
        # compute the instructions that the structure has to suposedly
        # complete until the structure stop. This is only to check
        # for collisions when computing the actual profile for the
        # horizontal motion (if needed).
        # Compute the actual time required to complete the instruction.
        sm.compute_time(instruction, next_instructions)
        # Simulate instruction:
        instruction_number += 1
        print(instruction_number, instruction)
        for res in sm.simulate_step(structure, instruction):
            if res == SimulatorState.SimulatorError:
                # The simulation has failed: Set to manual mode, to let the
                # user check the situation.
                graphics.set_manual_mode()
                graphics.draw(stairs, structure, sm, True)
                # Finish the outermost loop.
                # continue_loop = False
                break
            # The simulation has succeeded, so, continue loop.
            # elif res == SimulatorState.SimulatorNoIter:
            #     break
            continue_loop, key_pressed = \
                graphics.draw(stairs, structure, sm.counter)
            if not continue_loop or graphics.manual_mode:
                # The user has pressed the Esc key to finish the program.
                # Entering manual mode. Finish the inner while loop and
                # continue with a new iteration of the outermost while loop.
                # NOTE: At the end of the for loop, we substitute the current
                # structure by the one returned by the control module. However,
                # if we change mode before the instruction is finished, we
                # have to undo this step. This can easily be done changing
                # the last structure by the current one, so that when the
                # structure is substituted, it is substituted by the same.
                str_aux = structure
                break
    # if res == SimulatorState.SimulatorOK:
    #     continue_loop, key_pressed = graphics.draw(
    #         stairs, structure, sm.counter)
    # Substitute the simulated structure by the one returned by the control
    # module when computing the instruction in automatic mode. In manual mode,
    # both are just the same object.
    structure = str_aux

print("End of program.")
#
#
# ##########################################################################
# while continue_loop:
#     # To allow the program to enter the for loop at least once to give the
#     # user the chance to switch again to manual mode without doing nothing,
#     # just start automatic mode, send an empty instruction, that allow the fo
#     # loop to do one iteration without moving the structure.
#     instruction = {}
#     while graphics.manual_mode and continue_loop:
#         #######################################################################
#         #  Manual mode
#         #######################################################################
#         # Display image and wait for next instruction.
#         continue_loop, key_pressed = graphics.draw(stairs, structure, sm)
#
#         instruction = control.manual_control(key_pressed, sm)
#         if instruction is None:
#             continue
#         print("manual:", instruction)
#         for res in sm.simulate_instruction(structure, instruction):
#             pass
#     ###########################################################################
#     # Exiting manual mode and entering automatic mode.
#
#     str_aux = structure
#     while not graphics.manual_mode and continue_loop:
#         if instruction is None:
#             break
#         print("Inst", inst_number, ":",  instruction)
#         #######################################################################
#         #  Automatic mode
#         #######################################################################
#         inst_number += 1
#         # Allow the program to generate a new instruction in the next
#         # iteration
#         # switching_mode = False
# #         for res in sm.simulate_instruction(structure, instruction):
#         for res in sm.simulate_step(structure, instruction):
#             if not res:
#                 # The simulation has failed: Set to manual mode, to let the
#                 # user check the situation.
#                 graphics.set_manual_mode()
#                 graphics.draw(stairs, structure, sm, True)
#                 # Finish the outermost loop.
#                 # continue_loop = False
#                 break
#             # The simulation has succeeded, so, continue loop.
#             continue_loop, key_pressed = graphics.draw(stairs, structure, sm)
#             if not continue_loop or graphics.manual_mode:
#                 # The user has pressed the Esc key to finish the program.
#                 # Entering manual mode. Finish the inner while loop and
#                 # continue with a new iteration of the outermost while loop.
#                 # NOTE: At the end of the for loop, we substitute the current
#                 # structure by the one returned by the control module. Howeve
#                 # if we change mode before the instruction is finished, we
#                 # have to undo this step. This can easily be done changing
#                 # the last structure by the current one, so that when the
#                 # structure is substituted, it is substituted by the same.
#                 str_aux = structure
#                 break
#         # Substitute the simulated structure by the one returned by the
#         # control module.
#         structure = str_aux
#
#         if continue_loop and not graphics.manual_mode:
#             # Generate the next instruction.
#             try:
#                 # Compute the next instruction.
#                 instruction, str_aux = control.next_instruction(structure)
#                 if instruction is None:
#                     try:
#                         graphics.set_manual_mode()
#                     except ValueError:
#                         continue_loop = False
#                     continue
#                 stop_distance = sm.stop_distance(instruction)
#                 next_instructions = control.compute_distance(
#                     str_aux, stop_distance)
#                 # Compute and initial estimation of the time requirede to
#                 # complete the instruction, and if dynamics is implemented,
#                 # compute the instructions that the structure has to suposedl
#                 # complete until the structure stop. This is only to check
#                 # for collisions when computing the actual profile for the
#                 # horizontal motion (if needed).
#                 # Compute the actual time required to complete the instructio
#                 sm.compute_time(instruction, next_instructions)
#                 # str_aux2 = str_aux
#                 # while not sm.compute_time(instruction, next_instructions):
#                 #     next_instructions = control.compute_distance(
#                 #         str_aux2, instruction['stop_distance'])
#                 # next_inst, str_aux = control.next_instruction(str_aux2)
#                 # next_instructions += [next_inst]
#             except RuntimeError:
#                 # A runtime error is raised when the control does not find an
#                 # instruction (probably because the wheelchair has reached th
#                 # end of the stair). In this case, give the control to the
#                 # user to finish the program (or to do any other thing).
#                 graphics.set_manual_mode()
#             # NOTE: The last instruction returns the future state of the
#             # structure when the instruction were completed. This state would
#             # be the same than the structure would have after the instruction
#             # simulation that follows.
#             #
#             # However, due to rounding errors, this could be a little
#             # different, and can change the next instruction, and in some tim
#             # take one more or one less iteration to complete than if we do n
#             # simulate. for that reason, to make with or without simulation d
#             # the same instructions, we capture here the state at the end of
#             # the instruction, and after the simulation (see end of the
#             # previous for loop) substitute the simulated structure for this
#             # one.
#     ###########################################################################
# print("End of program.")

###############################################################################
# End of file.
###############################################################################
