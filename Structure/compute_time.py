'''
Created on 30 abr. 2021

@author: pedro

Module to compute the time required to complete a stair, without simulations.

'''

from structure import base
from control import control
from simulator import simulator

# Imports needed for the testing block.
import sys
from physics import stairs
import readXML


def compute_time(size, wheels, stairs, speed_data):
    """Return the time required to complete the stairs."""
    str_aux = base.Base(size, wheels, stairs)
    sim_aux = simulator.Simulator(speed_data)
    total_time = 0
    count = 0
    while True:
        instruction, str_aux = control.next_instruction(str_aux)
        total_time += sim_aux.compute_iterations(instruction)
        count += 1
        if instruction.get('end', False):
            break
    return total_time


if __name__ == "__main__":
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
    total = compute_time(structure_size, wheels_radius, stairs, speed_data)
    print("Total:", total)
    