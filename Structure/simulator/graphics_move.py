'''
Created on 22 mar. 2021

@author: dieg2
'''

import numpy as np
import cv2


class GraphMove:

    def __init__(self, size_gr):
        self.buffer_L1 = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_L2 = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_L3 = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_L4 = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_VS = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_AS = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer_AI = np.zeros(size_gr[0] - 100, dtype=float)
        self.buffer = np.zeros(size_gr[0] - 100, dtype=float)

    def GraphicMove(self, y, graphic_modify, actuador, index, size_gr,
                    y_lim_inf, y_lim_sup, divisiones):
        '''
        Esta funcion recibe como parametros de entrada:
        - Variable dependiente (y): seria la posicion de los actuadores para
                                    ese instante de tiempo
        - graphic_modify: seria la imagen grafica a modificar
        -actuador: toma un valor numerico de 1 a 6
            1: L1
            2: L2
            3: L3
            4: L4
            5: VS (Velocidad Estructura)
            6: AS (Aceleracion Estructura)
            7: AI (Inclinacion Estructura)
        -index: posicionador que se usara para indicar la muestra de mi buffer
        -size_gr: dimensiones de la imagen para la grafica
        -y_lim_inf: valor minimo para la y
        -y_lim_sup: valor maximo para la y
        -divisiones: numero de pixeles por division para el Grid

        Como dato de salida te devuelve la figura a capturar

        '''
        # Compruebo en que actuador debo guardar la posicion
        if actuador == 1:
            index = (index) % (len(self.buffer_L1))
            self.buffer_L1[index] = y
            index = (index+1) % (len(self.buffer_L1))
            self.buffer = np.hstack((self.buffer_L1[index:],
                                     self.buffer_L1[:index]))

        elif actuador == 2:
            index = (index) % (len(self.buffer_L2))
            self.buffer_L2[index] = y
            index = (index+1) % (len(self.buffer_L2))
            self.buffer = np.hstack((self.buffer_L2[index:],
                                     self.buffer_L2[:index]))

        elif actuador == 3:
            index = (index) % (len(self.buffer_L3))
            self.buffer_L3[index] = y
            index = (index+1) % (len(self.buffer_L3))
            self.buffer = np.hstack((self.buffer_L3[index:],
                                     self.buffer_L3[:index]))

        elif actuador == 4:
            index = (index) % (len(self.buffer_L4))
            self.buffer_L4[index] = y
            index = (index+1) % (len(self.buffer_L4))
            self.buffer = np.hstack((self.buffer_L4[index:],
                                     self.buffer_L4[:index]))
        elif actuador == 5:
            index = (index) % (len(self.buffer_VS))
            self.buffer_VS[index] = y
            index = (index+1) % (len(self.buffer_VS))
            self.buffer = np.hstack((self.buffer_VS[index:],
                                     self.buffer_VS[:index]))
        elif actuador == 6:
            index = (index) % (len(self.buffer_AS))
            self.buffer_AS[index] = y
            index = (index+1) % (len(self.buffer_AS))
            self.buffer = np.hstack((self.buffer_AS[index:],
                                     self.buffer_AS[:index]))
        elif actuador == 7:
            index = (index) % (len(self.buffer_AI))
            self.buffer_AI[index] = y
            index = (index+1) % (len(self.buffer_AI))
            self.buffer = np.hstack((self.buffer_AI[index:],
                                     self.buffer_AI[:index]))

        # Limpio el espacio de trabajo
        graphic_modify[48:(size_gr[1] - 50), 49:(size_gr[0] - 50)] = 0xFF
        # Dibujo de los ejes cartesianos y y x respectivamente para el actuador
        cv2.line(graphic_modify, (50, 50), (50, (size_gr[1] - 50)), (0, 0, 0),
                 1)
        cv2.line(graphic_modify, (50, (size_gr[1] - 50)),
                 ((size_gr[0] - 50), (size_gr[1] - 50)), (0, 0, 0), 1)

        # Vuelvo a construir el Grid
        # Numero de divisiones en x y en y
        num_div_x = int((size_gr[0] - 100) / divisiones)
        num_div_y = int((size_gr[1] - 100) / divisiones)

        for i in range(num_div_x+1):
            # Para el eje x
            delta_pixel_x = divisiones*i

            # Calulamos la posicion del pixel correspondiente
            pixel_x_g = 50 + delta_pixel_x

            # Elaboracion del Grid de lineas verticales
            if i > 0:
                cv2.line(graphic_modify, (pixel_x_g, 50),
                         (pixel_x_g, (size_gr[1] - 50)), (192, 192, 192), 1)

        for i in range(num_div_y+1):
            # Para el eje y
            delta_pixel_y = divisiones*i

            # Calulamos la posicion del pixel correspondiente
            pixel_y_g = (size_gr[1] - 50) - delta_pixel_y

            # Elaboracion del Grid de lineas  horizontales
            if i > 0:
                cv2.line(graphic_modify, (50, pixel_y_g),
                         ((size_gr[0] - 50), pixel_y_g), (192, 192, 192), 1)

        # Extremos finales del grid vertical y horizontal
        cv2.line(graphic_modify, (size_gr[0] - 50, 50),
                 (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

        cv2.line(graphic_modify, (50, 50),
                 (size_gr[0] - 50, 50), (192, 192, 192), 1)
        
        # Se dibuja la grafica leyendo el array de buffer
        #Pixeles auxiliares
        pixel_x_M1 = 50
        pixel_y_M1 = (size_gr[1] - 50) - int(((size_gr[1] - 100) /
                                            (y_lim_sup - y_lim_inf)) *
                                            (self.buffer[0] - y_lim_inf))
        
        for i in range(size_gr[0] - 100):
            # Variable independiente x
            pixel_x = 50 + i

            # Variable dependiente y
            pixel_y = (size_gr[1] - 50) - int(((size_gr[1] - 100) /
                                              (y_lim_sup - y_lim_inf)) *
                                              (self.buffer[i] - y_lim_inf))

            # Se pintan los pixeles correspondientes
            cv2.line(graphic_modify, (pixel_x_M1, pixel_y_M1), (pixel_x, pixel_y),
                     (25, 5, 140), 2)
            # graphic_modify[pixel_y+1, pixel_x+1] = [25, 5, 140]
            # graphic_modify[pixel_y, pixel_x] = [25, 5, 140]
            # graphic_modify[pixel_y-1, pixel_x-1] = [25, 5, 140]
            pixel_x_M1 = pixel_x
            pixel_y_M1 = pixel_y

        return graphic_modify
