"""
Created on 30 jun. 2021

@author: pedro.gil@uah.es

Module to compute times for the structure to cross a stair. This module
performs the same instructions as the graphical simulator but without any
graphics overload, to complete the function in less time. This module is
intended for the optimization functions.
"""

from control.control import next_instruction, compute_distance


def compute_time(structure, simulator):
    """Compute the time required to complete a stair.

    """
    total_time = 0
    while True:
        instruction, str_aux = next_instruction(structure)
        if instruction is None:
            break
        stop_distance = simulator.estimate_end_speed(instruction)
        next_instr = compute_distance(str_aux, stop_distance)
        simulator.compute_time(instruction, next_instr)
        total_time += instruction['time']
        structure = str_aux
        # Return the total number of iterations needed.
    return total_time


# class ComputeTime:
#     """Main class to compute times."""
#
#     def __init__(self, wheels, stairs_list, speed_data, dynamics_data):
#         """Constructor:
#
#         Arguments:
#         wheels -- Wheel radius.
#         stairs_list -- List of stair to check time.
#         speed_data -- Speeds for all the actuators and wheels.
#         """
#         # Check if the stairs is a tuple or a single stair.
#         if type(stairs_list) not in (list, tuple):
#             # If not, convert it to tuple.
#             stairs_list = (stairs_list,)
#         self.stairs = stairs_list
#         self.sim = simulator.Simulator(speed_data, dynamics_data)
#         self.wheels = wheels
#
#     def compute(self, size):
#         """Return the time required to complete the stairs."""
#
#         total_time = 0
#         for stair in self.stairs:
#             # Build the structure.
#             structure = base.Base(size, self.wheels, stair)
#             # Make a loop until the structure reaches the end of the stair.
#             while True:
#                 instruction, str_aux = next_instruction(structure)
#                 if instruction is None:
#                     break
#                 stop_distance = self.sim.estimate_end_speed(instruction)
#                 next_instr = compute_distance(str_aux, stop_distance)
#                 self.sim.compute_time(instruction, next_instr)
#                 total_time += instruction['time']
#                 # for res in self.sim.simulate_step(structure, instruction):
#                 #     pass
#                 structure = str_aux
#             # Return the total number of iterations needed.
#         return total_time
#
#     # def compute(self, size):
#     #     """Return the time required to complete the stairs."""
#     #
#     #     total_time = 0
#     #     instructions = []
#     #     for stair in self.stairs:
#     #         # Build the structure.
#     #         str_aux = base.Base(size, self.wheels, stair)
#     #         # Make a loop until the structure reaches the end of the stair.
#     #         while True:
#     #             instruction, str_aux = next_instruction(str_aux)
#     #             instructions += [instruction]
#     #             total_time += self.sim.compute_time(instructions[0])
#     #             instructions = instructions[1:]
#     #             if instruction.get('end', False):
#     #                 break
#     #         # Return the total number of iterations needed.
#     #     return total_time
