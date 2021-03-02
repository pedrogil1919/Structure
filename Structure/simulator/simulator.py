'''
Created on 28 ene. 2021

@author: pedro
'''

'''
Created on 5 jun. 2020

@author: pedro

Step by step simulator of structure motion.

'''
import numpy  

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
        self.str_up_speed_e = speed_data['struct_up']
        self.str_up_speed_i = speed_data['struct_up']
        self.str_dw_speed = speed_data['struct_down']
        
        
    def move_incline(self, T_total, a_h, Ts):
        if Ts*self.k < 0.3*T_total:
            #Ecuaciones para la altura
            a_k_i = a_h
            v_k_i = self.v_kM1_i + a_k_i*Ts
            delta_yk = v_k_i*Ts + (a_k_i*Ts**2.0)/2
            
        elif Ts*self.k >= 0.3*T_total and Ts*self.k < 0.7*T_total:
            #Ecuaciones para la altura
            a_k_i = 0
            v_k_i = self.v_kM1_i + a_k_i*Ts
            delta_yk = v_k_i*Ts
            
        elif Ts*self.k >= 0.7*T_total and Ts*self.k <= T_total:
            #Ecuaciones para la altura
            a_k_i = -a_h
            v_k_i = self.v_kM1_i + a_k_i*Ts
            delta_yk = v_k_i*Ts + (a_k_i*Ts**2.0)/2
            
        self.v_kM1_i = v_k_i
        return delta_yk
        
    def move_elevate(self, T_total, a_h, Ts):
        if Ts*self.k < 0.3*T_total:
            #Ecuaciones para la altura
            a_k_h = a_h
            v_k_h = self.v_kM1_h + a_k_h*Ts
            delta_yk = v_k_h*Ts + (a_k_h*Ts**2.0)/2
            
        elif Ts*self.k >= 0.3*T_total and Ts*self.k < 0.7*T_total:
            #Ecuaciones para la altura
            a_k_h = 0
            v_k_h = self.v_kM1_h + a_k_h*Ts
            delta_yk = v_k_h*Ts
            
        elif Ts*self.k >= 0.7*T_total and Ts*self.k <= T_total:
            #Ecuaciones para la altura
            a_k_h = -a_h
            v_k_h = self.v_kM1_h + a_k_h*Ts
            delta_yk = v_k_h*Ts + (a_k_h*Ts**2.0)/2
            
        self.v_kM1_h = v_k_h
        return delta_yk
        
    def move_desp (self, T_total, a_d, Ts):
        #Obtencion de la aceleracion, velocidad y delta_pos
        if Ts*self.k < 0.3*T_total:
            #Ecuaciones para el desplazamiento linel
            a_k_d = a_d
            v_k_d = self.v_kM1_d + a_k_d*Ts
            delta_xk = v_k_d*Ts + (a_k_d*Ts**2.0)/2
            
        elif Ts*self.k >= 0.3*T_total and Ts*self.k < 0.7*T_total:
            #Ecuaciones para el desplazamiento linel
            a_k_d = 0
            v_k_d = self.v_kM1_d + a_k_d*Ts
            delta_xk = v_k_d*Ts
         
        elif Ts*self.k >= 0.7*T_total and Ts*self.k <= T_total:
            #Ecuaciones para el desplazamiento linel
            a_k_d = -a_d 
            v_k_d = self.v_kM1_d + a_k_d*Ts
            delta_xk = v_k_d*Ts + (a_k_d*Ts**2.0)/2

            
        self.v_kM1_d = v_k_d
        return delta_xk
    
    def simulate_step(self, structure, instruction):
        """Complete a list of instructions in one step.

        """
        self.k = 0
        self.v_kM1_d = 0.0
        self.v_kM1_h = 0.0
        self.v_kM1_i = 0.0
        T_instruction = numpy.zeros(3, dtype = float)
        
        try:

            distance = instruction['advance']
            height_e = instruction['elevate']
            height_i = instruction['incline']
            
            #Calculo de los tiempos totales de elevar y avanzar
            #T_instruction[0]: tiempo que tarda para avnzar
            #T_instruction[1]: tiempo que tarda en elevar la estructura
            #T_instruction[2]: tiempo que tarda en inclinar la estructura
            T_instruction[0] = distance/(0.7*self.wheel_speed)
            T_instruction[1] = height_e/(0.7*self.str_up_speed_e)
            T_instruction[2] = height_i/(0.7*self.str_up_speed_i)
             
            if T_instruction[0] == max(T_instruction):
                Tt = T_instruction[0]
                self.str_up_speed_e = height_e/(0.7*Tt)
                self.str_up_speed_i = height_i/(0.7*Tt)
                
            elif T_instruction[1] == max(T_instruction):
                Tt = T_instruction[1]
                self.wheel_speed = distance/(0.7*Tt)
                self.str_up_speed_i = height_i/(0.7*Tt)
                
            elif T_instruction[2] == max(T_instruction):
                Tt = T_instruction[2]
                self.wheel_speed = distance/(0.7*Tt)
                self.str_up_speed_e = height_e/(0.7*Tt)
                
            #Calculo de la aceleracion maxima del sistema
            acel_d = self.wheel_speed/(0.3*Tt)
            acel_h_elevate = self.str_up_speed_e/(0.3*Tt)
            acel_h_incline = self.str_up_speed_i/(0.3*Tt)
            
            
            ts = 0.1

        except KeyError:
            pass
        else:

            # Advance structure
            res = structure.advance(distance)
            if not res:
                print("Can not advance structure.", res)
        #######################################################################  
        try:
            height = instruction['incline_prev']
        except KeyError:
            pass
        else:
            rear = instruction.get('elevate_rear', False)
#             front = instruction.get('fix_front', False)
            # Incline structure
            res = structure.incline(height, rear)
            if not res:
                print("Can not incline structure:", res)
        #######################################################################  
        try:
            height = instruction['elevate']
        except KeyError:
            pass
        else:
            # Elevate structure
            res = structure.elevate(height)
            if not res:
                print("Can not elevate structure.", res)
        #######################################################################  
        try:
            height = instruction['incline']
        except KeyError:
            pass
        else:

            #Instrucciones para la parte de inlcinar si se necesitara

            rear = instruction.get('elevate_rear', False)
            front = instruction.get('fix_front', False)
            # Advance structure
            while distance > 0 or height_e >0 or height_i > 0:
                #Desplazamiento horizontal
                d = self.move_desp(Tt, acel_d, ts)
                res, dis = structure.advance(d)
                #Elevacion de toda la estructura
                h_e = self.move_elevate(Tt, acel_h_elevate, ts)
                res, dis = structure.elevate(h_e)
                #Inclinacion de la estructura
                h_i = self.move_incline(Tt, acel_h_incline, ts)
                res, hor, ver = structure.incline(h_i, rear, front)
                
                distance -= d
                height_e -= h_e
                height_i -= h_i
                #Incremento el posicionador para la proxima iteracion
                self.k = self.k+1
                yield True
                if not res:
                    print("Can not advance structure. Error:", dis)

        #######################################################################  

        #######################################################################  
        # Check for the end of the trajectory.
        try:
            if instruction['end']:
                yield False
        except KeyError:
            pass
#         #######################################################################  
        yield True
        
    def simulate_instruction(self, structure, instruction):
        """Complete a list of instructions in one step.
 
        (for more details, see simulate_simple_step function).
 
        """
        try:
            distance = instruction['distance']
        except KeyError:
            pass
        else:
            # Advance structure
            res, dis = structure.advance(distance)
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
         

###############################################################################
# End of file.
###############################################################################        
        
