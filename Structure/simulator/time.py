"""
Created on 30 jun. 2021

@author: pedro.gil@uah.es

Module to compute times for the structure to cross a stair. This module
performs the same instructions as the graphical simulator but without any
graphics overload, to complete the function in less time. This module is
intended for the optimization functions.
"""

from structure import base
from simulator import simulator
from control.control import next_instruction


class ComputeTime:
    """Main class to compute times."""

    def __init__(self, wheels, stairs_list, speed_data):
        """Constructor:

        Arguments:
        wheels -- Wheel radius.
        stairs_list -- List of stair to check time.
        speed_data -- Speeds for all the actuators and wheels.
        """
        # Check if the stairs is a tuple or a single stair.
        if type(stairs_list) not in (list, tuple):
            # If not, convert it to tuple.
            stairs_list = (stairs_list,)
        self.stairs = stairs_list
        self.sim = simulator.Simulator(speed_data)
        self.wheels = wheels

    def compute(self, size):
        """Return the time required to complete the stairs."""

        total_time = 0
        for stair in self.stairs:
            # Build the structure.
            str_aux = base.Base(size, self.wheels, stair)
            # Make a loop until the structure reaches the end of the stair.
            while True:
                instruction, str_aux = next_instruction(str_aux)
                total_time += self.sim.compute_iterations(instruction)
                if instruction.get('end', False):
                    break
            # Return the total number of iterations needed.
        return total_time
