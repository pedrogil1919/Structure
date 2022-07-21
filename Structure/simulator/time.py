"""
Created on 30 jun. 2021

@author: pedro.gil@uah.es

Module to compute times for the structure to cross a stair. This module
performs the same instructions as the graphical simulator but without any
graphics overload, to complete the function in less time. This module is
intended for the optimization functions.

If the structure can not complete the whole stair, the exception is not caugth
here. Remenber that all the exceptions returned but the RunTimeError are
produced because the system can not complete the motion. Only RunTimeError
exceptions means some error within the code that must be corrected.

"""

from simulator.control import next_instruction, compute_distance


def compute_time(structure, simulator):
    """Compute the time required to complete a stair.

    """
    total_time = 0.0
    # List of instructions to complete the stair. This is a FIFO queue. In
    # general, to execute a instruction, we need more instructions, just for
    # the control to check if a crash can happen if the speed at the end of the
    # current instruction is high enogh so that the structure can not stop
    # before the crash.
    instructions = []
    while True:
        try:
            # Check if we have at least one instruction in the queue.
            instruction = instructions[0]
            str_aux = instruction['struct']
            # And remove this instruction from the list.
            instructions = instructions[1:]
        except IndexError:
            # In case the list is empty, compute just the next instruction.
            instruction, str_aux = next_instruction(structure)
        if instruction is None:
            # This means that the control module can not find a valid
            # instruction, and so, the stair can not be crossed.
            raise ValueError("Stair can not be crossed")
        # To fix the end speed for the current instruction, we have to chek for
        # any possible crash in the following instructions. For that reason, we
        # need to compute first the stop distance.
        stop_distance = simulator.stop_distance(instruction)
        # And now, we compute the instructions to complete that distance.
        instructions = compute_distance(str_aux, stop_distance, instructions)
        # And with all these, compute the end speed, and so, the time required
        # for the current instruction.
        simulator.compute_time(instruction, instructions)
        # Add this time to the total time.
        total_time += instruction['time']
        # And update the state of the structure with the current state.
        structure = str_aux
        # Check if we have finished the stair.
        if instruction.get("end", False):
            # When the instruction includes the key "end", that means that we
            # have complete the stair.
            break
        # Return the total number of iterations needed.
    return total_time

###############################################################################
# End of file.
###############################################################################

# def compute_time1(structure, simulator):
#     """Similar to above, but without optimization, that is here, the same
#     instruction is computed many times.
#
#     """
#     total_time = 0
#     while True:
#         instruction, str_aux = next_instruction(structure)
#         if instruction is None:
#             break
#         stop_distance = simulator.stop_distance(instruction)
#         next_instr = compute_distance(str_aux, stop_distance, False)
#         simulator.compute_time(instruction, next_instr)
#         total_time += instruction['time']
#         structure = str_aux
#         # Return the total number of iterations needed.
#     return total_time


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
