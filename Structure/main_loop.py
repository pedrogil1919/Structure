'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

'''

"probar token"
"otro"

import sys

from physics import stairs
from structure import base
from control import control
from simulator.simulator import Simulator
# from simulators.simulator import simulate_complete as simulate
# from simulators.simulator import simulate_instruction as simulate
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
structure = base.Base(structure_size, wheels_radius, stairs)
# Read simulator data.
speed_data = readXML.read_simulator(settings_name)
sm = Simulator(speed_data)
# Read graphical variables.
image_data, video_data = readXML.read_graphics(settings_name)
graphics = Graphics(image_data, video_data)
continue_loop, manual_mode, key_pressed = graphics.draw(stairs, structure)

# Continue_loop is a flag to help finish the program. It gets False value when
# the user press the Esc key (see graphics module).
# Main loop
inst_number = 0

while continue_loop:
    while manual_mode and continue_loop:
    ###########################################################################
    #  Manual mode
    ###########################################################################
        # Display image and wait for next instruction.
        continue_loop, manual_mode, key_pressed = \
            graphics.draw(stairs, structure)
        instruction = control.manual_control(key_pressed, sm)
        print("manual:", instruction)
        for res in sm.simulate_instruction(structure, instruction):
            pass
    ###########################################################################
    # Exiting manual mode and entering automatic mode.

    # To allow the program to enter the for loop at least once to give the
    # user the chance to switch again to manual mode without doing nothing, or
    # just start automatic mode, send an empty instruction, that allow the for
    # loop to do one iteration without moving the structure.
    instruction = {}
    # However, we need that the control module does not produce any instruction,
    # so that the structure does not move until the user press the key. This
    # flag prevent this module to produce any instruction. After first
    # iteration, the flag is set to True to allow the module to generate the
    # first instruction.
    switching_mode = True
    
    while not manual_mode and continue_loop:
    ###########################################################################
    #  Automatic mode
    ###########################################################################
        if not switching_mode:
            instruction = control.next_instruction(structure)
          
            print("Inst", inst_number, ":",  instruction)
            if inst_number == 48:
                print("in")
            inst_number += 1
        # Allow the program to generate a new instruction in the next
        # iteration
        switching_mode = False
#         for res in sm.simulate_instruction(structure, instruction):
        for res in sm.simulate_step(structure, instruction):
            if not res:
                # The simulation has finished or failed: finish the program.
                print("Press any key to finish...")
                graphics.draw(stairs, structure, True)
                # Finish the outermost loop.
                continue_loop = False
                break
            # The simulation has succeeded, so, continue loop.
            continue_loop, manual_mode, key_pressed = \
                    graphics.draw(stairs, structure)
            if not continue_loop:
                # The user has pressed the Esc key to finish the program.
                break
            if manual_mode:
                # Entering manual mode. Finish the inner while loop an continue
                # with a new iteration of the outermost while loop.
                break
    ###########################################################################    
print("End of program.")

###############################################################################
# End of file.
###############################################################################
