"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

"""

import sys

from physics import stairs
from structure import base
from control import control
from simulator.simulator import Simulator
from simulator.graphics import Graphics
import readXML

# Open and check settings file.
try:
    settings_name = sys.argv[1]
except Exception:
    settings_name = "settings.xml"

# Read stairs data and create physical stairs object.
stairs_list, landing = readXML.read_stairs(settings_name)
stairs = stairs.Stair(stairs_list, landing)
# Read structure dimensions and create structure.
structure_size, wheels_radius = readXML.read_structure(settings_name)
structure = base.Base(structure_size, wheels_radius, stairs)  # , graphics)

# Read simulator data.
speed_data = readXML.read_simulator(settings_name)
sm = Simulator(speed_data)

# Read graphical variables.
image_data, video_data = readXML.read_graphics(settings_name)
axis = {
    "height": structure_size["d"],
    "max_speed": 1.2 * speed_data["wheel"]}
graphics = Graphics(image_data, video_data, axis)
# Draw initial state of the structure.
continue_loop, key_pressed = graphics.draw(stairs, structure)
# Continue_loop is a flag to help finish the program. It gets False value when
# the user press the Esc key (see graphics module).
# Main loop
inst_number = 0

while continue_loop:
    # To allow the program to enter the for loop at least once to give the
    # user the chance to switch again to manual mode without doing nothing, or
    # just start automatic mode, send an empty instruction, that allow the for
    # loop to do one iteration without moving the structure.
    instruction = {}
    while graphics.manual_mode and continue_loop:
        #######################################################################
        #  Manual mode
        #######################################################################
        # Display image and wait for next instruction.
        continue_loop, key_pressed = graphics.draw(stairs, structure)

        instruction = control.manual_control(key_pressed, sm)
        print("manual:", instruction)
        for res in sm.simulate_instruction(structure, instruction):
            pass
    ###########################################################################
    # Exiting manual mode and entering automatic mode.

    str_aux = structure
    while not graphics.manual_mode and continue_loop:
        print("Inst", inst_number, ":",  instruction)
        #######################################################################
        #  Automatic mode
        #######################################################################
        inst_number += 1
        # Allow the program to generate a new instruction in the next
        # iteration
        # switching_mode = False
#         for res in sm.simulate_instruction(structure, instruction):
        for res in sm.simulate_step(structure, instruction):
            if not res:
                # The simulation has finished or failed: finish the program.
                # print("Press any key to finish...")
                graphics.set_manual_mode()
                graphics.draw(stairs, structure, True)
                # Finish the outermost loop.
                # continue_loop = False
                break
            # The simulation has succeeded, so, continue loop.
            continue_loop, key_pressed = graphics.draw(stairs, structure)
            if not continue_loop:
                # The user has pressed the Esc key to finish the program.
                break
            if graphics.manual_mode:
                # Entering manual mode. Finish the inner while loop an continue
                # with a new iteration of the outermost while loop.
                # NOTE: At the end of the for loop, we substitute the current
                # structure by the one returned by the control module. However,
                # if we change mode before the instruction is finished, we
                # have to undo this step. This can easily be done changing
                # the last structure by the current one, so that when the
                # structure is substituted, it is substituted by the same.
                str_aux = structure
                break
        # Substitute the simulated structure by the one returned by the
        # control module.
        structure = str_aux

        if continue_loop and not graphics.manual_mode:
            # Generate the next instruction.
            try:
                instruction, str_aux = control.next_instruction(structure)
            except RuntimeError:
                # A runtime error is raised when the control does not find any
                # instruction (probably because the wheelchair has reached the
                # end of the stair). In this case, give the control to the
                # user to finish the program (or to do any other thing).
                graphics.set_manual_mode()
            # NOTE: The last instruction returns the future state of the
            # structure when the instruction were completed. This state would
            # be the same than the structure would have after the instruction
            # simulation that follows.
            #
            # However, due to rounding errors, this could be a little
            # different, and can change the next instruction, and in some times
            # take one more or one less iteration to complete than if we do not
            # simulate. for that reason, to make with or without simulation do
            # the same instructions, we capture here the state at the end of
            # the instruction, and after the simulation (see end of the
            # previous for loop) substitute the simulated structure for this
            # one.
    ###########################################################################
print("End of program.")

###############################################################################
# End of file.
###############################################################################
