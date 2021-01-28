'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

'''
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
simulator = Simulator(speed_data)
# Read graphical variables.
image_data, video_data = readXML.read_graphics(settings_name)
graphics = Graphics(image_data, video_data)
continue_loop, manual_mode, key_pressed = graphics.draw(stairs, structure)

# Main loop
while continue_loop:
    ###########################################################################
    # Manual mode. Do the loop until the user press the Tab key.
    while manual_mode and continue_loop:
        # Display image and whait for next instruction.
        continue_loop, manual_mode, key_pressed = \
            graphics.draw(stairs, structure)
        instruction = control.manual_control(key_pressed, simulator)
        # Execute user instruction. Note that the required motion can be
        # greater than the motion performed in one step of the simulator. Since
        # we are in manual mode, we have to perform the whole motion in one
        # step. Otherwise, the user must press a key several times until one
        # instruction is completed, what is not a correct performance. To
        # avoid that, the whole instruction is inside a for loop, which end
        # when the whole motion is performed.
        for res in simulator.simulate_instruction(structure, instruction):
            pass
    ###########################################################################
print("End")
