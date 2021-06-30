"""
Created on 30 abr. 2021

@author: pedro

Module to compute the time required to complete a stair, without simulations.

"""

from simulator.time import ComputeTime

# Imports needed for the testing block.
import sys
from physics.stairs import Stair
import readXML

# Open and check settings file.
try:
    settings_name = sys.argv[1]
except Exception:
    settings_name = "settings.xml"

# Read stairs data and create physical stairs object.
stairs_list, landing = readXML.read_stairs(settings_name)
stair = Stair(stairs_list, landing)

for st in stairs_list:
    st['h'] = -st['h']
stair2 = Stair(stairs_list, landing)
stair = (stair, stair2)

# Read structure dimensions and create structure.
structure_size, wheels_radius = readXML.read_structure(settings_name)
# Read simulator data.
speed_data = readXML.read_simulator(settings_name)
compute_time = ComputeTime(wheels_radius, stair, speed_data)
total = compute_time.compute(structure_size)
print("Total:", total)
