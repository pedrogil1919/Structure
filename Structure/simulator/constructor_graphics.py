'''
Created on 22 mar. 2021

@author: dieg2
'''
import numpy
import cv2
from simulator import graphics_move


def constructor_graph(size_gr, y_lim_inf, y_lim_sup, x_lim_inf, x_lim_sup,
                      v_lim_inf, v_lim_sup, a_lim_inf, a_lim_sup,
                      gi_lim_inf, gi_lim_sup, divisiones, image_gr_L1,
                      image_gr_L2, image_gr_L3, image_gr_L4, image_gr_VS,
                      image_gr_AS, image_gr_AI):

    # Numero de divisiones en x y en y
    num_div_x = int((size_gr[0] - 100) / divisiones)
    num_div_y = int((size_gr[1] - 100) / divisiones)

    eje_x = numpy.array(numpy.zeros(num_div_x+1))
    eje_y = numpy.array(numpy.zeros(num_div_y+1))
    eje_y_v = numpy.array(numpy.zeros(num_div_y+1))
    eje_y_a = numpy.array(numpy.zeros(num_div_y+1))
    eje_y_gi = numpy.array(numpy.zeros(num_div_y+1))

    # Me declaro el array de valores a escribir en las graficas
    for i in range(num_div_x+1):
        eje_x[i] = int(x_lim_inf + ((x_lim_sup - x_lim_inf) /
                                    (size_gr[0] - 100)) * divisiones * i)

    for i in range(num_div_y+1):
        eje_y[i] = int(y_lim_inf + ((y_lim_sup - y_lim_inf) /
                                    (size_gr[1] - 100)) * divisiones * i)
        eje_y_v[i] = int(v_lim_inf + ((v_lim_sup - v_lim_inf) /
                                      (size_gr[1] - 100)) * divisiones * i)
        eje_y_a[i] = int(a_lim_inf + ((a_lim_sup - a_lim_inf) /
                                      (size_gr[1] - 100)) * divisiones * i)
        eje_y_gi[i] = int(gi_lim_inf + ((gi_lim_sup - gi_lim_inf) /
                                        (size_gr[1] - 100)) * divisiones * i)

    image_gr_L1[:] = 0xFF
    image_gr_L2[:] = 0xFF
    image_gr_L3[:] = 0xFF
    image_gr_L4[:] = 0xFF
    image_gr_VS[:] = 0xFF
    image_gr_AS[:] = 0xFF
    image_gr_AI[:] = 0xFF
    # Creacion de la grafica
    # Dibujo del rectangulo o cajetin
    cv2.rectangle(image_gr_L1, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_L2, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_L3, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_L4, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_VS, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_AS, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    cv2.rectangle(image_gr_AI, (10, 25), (size_gr[0] - 10, size_gr[1] - 10),
                  (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para L1
    cv2.line(image_gr_L1, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_L1, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para L2
    cv2.line(image_gr_L2, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_L2, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para L3
    cv2.line(image_gr_L3, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_L3, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para L4
    cv2.line(image_gr_L4, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_L4, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para VS
    cv2.line(image_gr_VS, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_VS, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para AS
    cv2.line(image_gr_AS, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_AS, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Dibujo de los ejes cartesianos y y x respectivamente para AI
    cv2.line(image_gr_AI, (50, 50), (50, size_gr[1] - 50), (0, 0, 0),
             1)
    cv2.line(image_gr_AI, (50, size_gr[1] - 50),
             (size_gr[0] - 50, size_gr[1] - 50), (0, 0, 0), 1)
    # Titulo de cada grafica
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image_gr_L1, 'Actuador L1', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_L2, 'Actuador L2', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_L3, 'Actuador L3', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_L4, 'Actuador L4', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_VS, 'Velocidad Estructura', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_AS, 'Aceleracion Estructura', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    cv2.putText(image_gr_AI, 'Inclinacion Estructura', (10, 20), font, 0.75,
                (0, 0, 0), 2)
    # Texto identificador de cada eje
    cv2.putText(image_gr_L1, 'Posicion (cm)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L2, 'Posicion (cm)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L3, 'Posicion (cm)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L4, 'Posicion (cm)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_VS, 'Velocidad (cm/s)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_AS, 'Aceleracion (cm/s^2)', (25, 37), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_AI, 'Inclinacion (cm)', (25, 37), font, 0.35,
                (0, 0, 0), 1)

    cv2.putText(image_gr_L1, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L2, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L3, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_L4, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_VS, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_AS, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    cv2.putText(image_gr_AI, 'Tiempo (s)',
                (size_gr[0] - 100, size_gr[1] - 15), font, 0.35,
                (0, 0, 0), 1)
    # Numeracion de los ejes coordenados
    # Numeracion para el eje x y grid vertical
    for i in range(num_div_x+1):
        # Para el eje x
        delta_pixel_x = divisiones

        # Calulamos la posicion del pixel correspondiente
        pixel_x = 50 + delta_pixel_x*i

        # Convertimos a string el valor correspondiente
        num_x = str(eje_x[i])

        # Escribismo los datos equiespaciados en los ejes x
        cv2.putText(image_gr_L1, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L2, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L3, num_x,
                    (pixel_x, (size_gr[1] - 50)+15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L4, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_VS, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_AS, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_AI, num_x,
                    (pixel_x, (size_gr[1] - 50) + 15),
                    font, 0.35, (0, 0, 0), 1)

        # Elaboracion del Grid de lineas verticales
        if i > 0:
            cv2.line(image_gr_L1, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_L2, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_L3, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_L4, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_VS, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_AS, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

            cv2.line(image_gr_AI, (pixel_x, 50),
                     (pixel_x, size_gr[1] - 50), (192, 192, 192), 1)

    # Numeracion para el eje y y grid horizontal
    for i in range(num_div_y+1):
        # Para el eje y
        delta_pixel_y = divisiones

        # Calulamos la posicion del pixel correspondiente
        pixel_y = (size_gr[1] - 50) - delta_pixel_y*i

        # Convertimos a string el valor correspondiente
        num_y = str(eje_y[i])
        num_v = str(eje_y_v[i])
        num_a = str(eje_y_a[i])
        num_gi = str(eje_y_gi[i])

        # Escribismo los datos equiespaciados en los ejes y
        cv2.putText(image_gr_L1, num_y, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L2, num_y, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L3, num_y, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_L4, num_y, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_VS, num_v, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_AS, num_a, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)

        cv2.putText(image_gr_AI, num_gi, (13, pixel_y),
                    font, 0.35, (0, 0, 0), 1)
        # Elaboracion del Grid de lineas verticales
        if i > 0:
            cv2.line(image_gr_L1, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_L2, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_L3, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_L4, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_VS, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_AS, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

            cv2.line(image_gr_AI, (50, pixel_y),
                     (size_gr[0] - 50, pixel_y), (192, 192, 192), 1)

    # Dibujo los extremos finales del grid de las graficas e incluyo los
    # valores maximos en cada eje
    # Para el eje x
    cv2.line(image_gr_L1, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_L2, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_L3, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_L4, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_VS, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_AS, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)

    cv2.line(image_gr_AI, (size_gr[0] - 50, 50),
             (size_gr[0] - 50, size_gr[1] - 50), (192, 192, 192), 1)
    # Escribimos el texto de los valores maximos en x
    # cv2.putText(image_gr_L1, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50) + 15),
                # font, 0.35, (0, 0, 0), 1)
                #
    # cv2.putText(image_gr_L2, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50) + 15),
                # font, 0.35, (0, 0, 0), 1)
                #
    # cv2.putText(image_gr_L3, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50) + 15),
                # font, 0.35, (0, 0, 0), 1)
                #
    # cv2.putText(image_gr_L4, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50) + 15),
                # font, 0.35, (0, 0, 0), 1)
                #
    # cv2.putText(image_gr_VS, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50)+15),
                # font, 0.35, (0, 0, 0), 1)
                #
    # cv2.putText(image_gr_AS, str(x_lim_sup),
                # (size_gr[0] - 50, (size_gr[1] - 50) + 15),
                # font, 0.35, (0, 0, 0), 1)
    
    # Para el eje y
    cv2.line(image_gr_L1, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_L2, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_L3, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_L4, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_VS, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_AS, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)

    cv2.line(image_gr_AI, (50, 50),
             (size_gr[0] - 50, 50), (192, 192, 192), 1)
    # Escribimos el texto de los valores maximos y del cero en y
    cv2.putText(image_gr_L1, str(y_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_L2, str(y_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_L3, str(y_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_L4, str(y_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_VS, str(v_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_AS, str(a_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    cv2.putText(image_gr_AI, str(gi_lim_sup), (13, 50),
                font, 0.35, (0, 0, 0), 1)

    return (image_gr_L1, image_gr_L2, image_gr_L3, image_gr_L4, image_gr_VS,
            image_gr_AS, image_gr_AI)


class Plots():

    def __init__(self, size):
        plot_data = {
            ##################################
            # Estas lineas son para el grafico
            'size_graph': size,
            'divisiones': 50,
            'y_lim_inf': 0,
            'y_lim_sup': 80,
            'x_lim_inf': 0,
            'v_lim_inf': 0,
            'v_lim_sup': 20,
            'a_lim_inf': -50,
            'a_lim_sup': 50,
            'gi_lim_inf': -80,
            'gi_lim_sup': 80}

        # Instruccuiones para las graficas
        self.size_gr = plot_data['size_graph']
        self.pix_div = plot_data['divisiones']
        self.y_lim_inf = plot_data['y_lim_inf']
        self.y_lim_sup = plot_data['y_lim_sup']
        self.x_lim_inf = plot_data['x_lim_inf']
        self.Ts = 0.1
        self.x_lim_sup = (self.size_gr[0] - 100) * self.Ts
        self.v_lim_inf = plot_data['v_lim_inf']
        self.v_lim_sup = plot_data['v_lim_sup']
        self.a_lim_inf = plot_data['a_lim_inf']
        self.a_lim_sup = plot_data['a_lim_sup']
        self.gi_lim_inf = plot_data['gi_lim_inf']  # Grados de la inclinacion
        self.gi_lim_sup = plot_data['gi_lim_sup']

        # Llamo al constriuctor de graphics_move
        self.GM = graphics_move.GraphMove(self.size_gr)

        self.image_gr_L1 = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_L2 = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_L3 = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_L4 = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_VS = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_AS = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)
        self.image_gr_AI = numpy.full((self.size_gr[1], self.size_gr[0], 3),
                                      0xFF, numpy.uint8)

        [self.image_gr_L1,
         self.image_gr_L2,
         self.image_gr_L3,
         self.image_gr_L4,
         self.image_gr_VS,
         self.image_gr_AS,
         self.image_gr_AI] = constructor_graph(self.size_gr,
                                               self.y_lim_inf,
                                               self.y_lim_sup,
                                               self.x_lim_inf,
                                               self.x_lim_sup,
                                               self.v_lim_inf,
                                               self.v_lim_sup,
                                               self.a_lim_inf,
                                               self.a_lim_sup,
                                               self.gi_lim_inf,
                                               self.gi_lim_sup,
                                               self.pix_div,
                                               self.image_gr_L1,
                                               self.image_gr_L2,
                                               self.image_gr_L3,
                                               self.image_gr_L4,
                                               self.image_gr_VS,
                                               self.image_gr_AS,
                                               self.image_gr_AI)

    def save(self, structure, counter):

        figura_L1 = self.GM.GraphicMove(structure.get_actuators_position(0),
                                        self.image_gr_L1, 1, counter,
                                        self.size_gr, self.y_lim_inf,
                                        self.y_lim_sup, self.pix_div)
        figura_L2 = self.GM.GraphicMove(structure.get_actuators_position(1),
                                        self.image_gr_L2, 2, counter,
                                        self.size_gr, self.y_lim_inf,
                                        self.y_lim_sup, self.pix_div)
        figura_L3 = self.GM.GraphicMove(structure.get_actuators_position(2),
                                        self.image_gr_L3, 3, counter,
                                        self.size_gr, self.y_lim_inf,
                                        self.y_lim_sup, self.pix_div)
        figura_L4 = self.GM.GraphicMove(structure.get_actuators_position(3),
                                        self.image_gr_L4, 4, counter,
                                        self.size_gr, self.y_lim_inf,
                                        self.y_lim_sup, self.pix_div)
        figura_v_struct = self.GM.GraphicMove(structure.get_speed(),
                                              self.image_gr_VS, 5,
                                              counter, self.size_gr,
                                              self.v_lim_inf,
                                              self.v_lim_sup,
                                              self.pix_div)
        figura_AI_struct = self.GM.GraphicMove(structure.get_inclination(),
                                               self.image_gr_AI, 7,
                                               counter, self.size_gr,
                                               self.gi_lim_inf,
                                               self.gi_lim_sup,
                                               self.pix_div)

        return figura_L1, figura_L2, figura_L3, figura_L4, figura_v_struct, figura_AI_struct
    