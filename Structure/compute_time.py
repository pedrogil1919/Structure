"""
Created on 30 abr. 2021

@author: pedro

Module to compute the time required to complete a stair, without simulations.

"""

# Imports needed for the testing block.
import sys

from control.control import ControlError
from structure.base import Base
from simulator.simulator import Simulator
from simulator.time import compute_time
from physics.stairs import Stair
import readXML


def check_max_size(size, wheels):
    """Example of ose of limits argument of Base.

    To check any constrain, simple code here all checkings, and raise a
    ValueError if not valid.

    """
    if size['a'] + size['b'] + size['c'] > 150.0:
        raise ValueError


# Open and check settings file.
try:
    settings_name = sys.argv[1]
except Exception:
    settings_name = "settings.xml"

# Read stairs data and create physical stairs object.
stairs_list, landing = readXML.read_stairs(settings_name)
stair = Stair(stairs_list, landing)

# Read structure dimensions and create structure.
structure_size, wheels_radius = readXML.read_structure(settings_name)
structure = Base(structure_size, wheels_radius, stair, check_max_size)
# Read simulator data.
dynamics_data, sample_data = readXML.read_dynamics(settings_name)
simulator = Simulator(dynamics_data, sample_data)

try:
    total_time = compute_time(structure, simulator)
    # compute_time = ComputeTime(
    #     wheels_radius, stair, dynamics_data, sample_data)
    # total = compute_time.compute(structure_size)
except ControlError:
    print("Can not cross the stair.")
else:
    print("Total:", total_time, "seconds")
