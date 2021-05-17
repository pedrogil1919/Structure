'''
Created on 30 abr. 2021

@author: pedro

Module to compute the time required to complete a stair, without simulations.

'''

from control import control

# Imports needed for the testing block.
import sys
from physics import stairs
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
# Read simulator data.
speed_data = readXML.read_simulator(settings_name)
total = control.compute_time(structure_size, wheels_radius, stairs, speed_data)
print("Total:", total)
