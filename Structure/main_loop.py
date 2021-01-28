'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

'''
import sys

from physics import stairs
from structure import base
from control import control
from simulator import simulator
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
sm = simulator.Simulator(speed_data)
# Read graphical variables.
image_data, video_data = readXML.read_graphics(settings_name)
graphics = Graphics(image_data, video_data)
continue_loop, manual_mode, key_pressed = graphics.draw(stairs, structure)

# Main loop
while continue_loop:
    # Display image and whait for next instruction.
    continue_loop, manual_mode, key_pressed = \
        graphics.draw(stairs, structure)
    instruction = control.manual_control(key_pressed, sm)
    sm.simulate_instruction(structure, instruction)
    ###########################################################################
print("End")
