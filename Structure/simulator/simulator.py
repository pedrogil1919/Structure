'''
Created on 28 ene. 2021

@author: pedro
'''

'''
Created on 5 jun. 2020

@author: pedro

Step by step simulator of structure motion.

'''


class Simulator():
    """Class to simulate structure motion.

    """
    def __init__(self, speed_data):
        """
        Constructor:

        Parameters:
        speed_data: dictionary with the following keys:
            wheel -- Structure horizontal speed, in units/frame.
            actuator -- Actuator shift speed (without load), in units/frame.
            elevate -- Structure elevate/incline (with load), in units/frame.
            TODO: When the structure is taken down, its speed is the
                actuator speed (no load). Check if this is correct.
        """
        self.wheel_speed = speed_data['wheel']
        self.actuator_speed = speed_data['actuator']
        self.str_up_speed = speed_data['struct_up']
        self.str_dw_speed = speed_data['struct_down']
        
    def move (self,T_total,a,Ts):
        #Obtencion de la aceleracion, velocidad y delta_pos
        if Ts*self.k < 0.3*T_total:
            a_k = a
            v_k = self.v_kM1 + a_k*Ts
            delta_xk = v_k*Ts + (a_k*Ts**2.0)/2
        elif Ts*self.k >= 0.3*T_total and Ts*self.k < 0.7*T_total:
            a_k = 0
            v_k = self.v_kM1 + a_k*Ts
            delta_xk = v_k*Ts
        elif Ts*self.k >= 0.7*T_total and Ts*self.k <= T_total:
            a_k = -a 
            v_k = self.v_kM1 + a_k*Ts
            delta_xk = v_k*Ts + (a_k*Ts**2.0)/2
            
        self.v_kM1 = v_k
        self.k = self.k+1
        
        return delta_xk
    
    def simulate_instruction(self, structure, instruction):
        """Complete a list of instructions in one step.

        (for more details, see simulate_simple_step function).

        """
        self.k = 0
        self.v_kM1 = 0.0
        
        try:
            distance = instruction['distance']
            Tt = distance/(0.7*self.wheel_speed)
            acel = self.wheel_speed/(0.3*Tt)
            ts = 0.1
        except KeyError:
            pass
        else:
            # Advance structure
            while distance > 0:
                d = self.move(Tt,acel,ts)
                res, dis = structure.advance(d)
                distance -= d
                yield True
                if not res:
                    print("Can not advance structure. Error:", dis)
        #######################################################################  
        try:
            height = instruction['elevate']
        except KeyError:
            pass
        else:
            # Elevate structure
            res, dis = structure.elevate(height)
            if not res:
                print("Can not elevate structure. Error:", dis)
        #######################################################################  
        try:
            height = instruction['incline']
        except KeyError:
            pass
        else:
            rear = instruction.get('elevate_rear', False)
            front = instruction.get('fix_front', False)
            # Incline structure
            res, hor, ver = structure.incline(height, rear, front)
            if not res:
                print("Can not incline structure:")
                print(" Vertical:", ver)
                print(" Horizontal:", hor)
        #######################################################################  
        try:
            height = instruction['shift']
            wheel = instruction['wheel']
        except KeyError:
            pass
        else:
            # Shift actuator.
            res, dis = structure.shift_actuator(wheel, height)
            if not res:
                print("Can not shift actuator. Error:", dis)
        #######################################################################  
        # Check for the end of the trajectory.
        try:
            if instruction['end']:
                yield False
        except KeyError:
            pass
        #######################################################################  
        yield True
        
#     def simulate_instruction(self, structure, instruction):
#         """Complete a list of instructions in one step.
# 
#         (for more details, see simulate_simple_step function).
# 
#         """
#         try:
#             distance = instruction['distance']
#         except KeyError:
#             pass
#         else:
#             # Advance structure
#             res, dis = structure.advance(distance)
#             if not res:
#                 print("Can not advance structure. Error:", dis)
#         #######################################################################  
#         try:
#             height = instruction['elevate']
#         except KeyError:
#             pass
#         else:
#             # Elevate structure
#             res, dis = structure.elevate(height)
#             if not res:
#                 print("Can not elevate structure. Error:", dis)
#         #######################################################################  
#         try:
#             height = instruction['incline']
#         except KeyError:
#             pass
#         else:
#             rear = instruction.get('elevate_rear', False)
#             front = instruction.get('fix_front', False)
#             # Incline structure
#             res, hor, ver = structure.incline(height, rear, front)
#             if not res:
#                 print("Can not incline structure:")
#                 print(" Vertical:", ver)
#                 print(" Horizontal:", hor)
#         #######################################################################  
#         try:
#             height = instruction['shift']
#             wheel = instruction['wheel']
#         except KeyError:
#             pass
#         else:
#             # Shift actuator.
#             res, dis = structure.shift_actuator(wheel, height)
#             if not res:
#                 print("Can not shift actuator. Error:", dis)
#         #######################################################################  
#         # Check for the end of the trajectory.
#         try:
#             if instruction['end']:
#                 yield False
#         except KeyError:
#             pass
#         #######################################################################  
#         yield True
#         

###############################################################################
# End of file.
###############################################################################        
        
