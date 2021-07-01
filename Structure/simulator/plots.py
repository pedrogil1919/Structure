"""
Created on 31 may. 2021

@author: pedro.gil@uah.es

Plotting variables as a function of time using matplotlib.
"""

# TODO: Document this class, if needed.

import io
import numpy
import cv2
import matplotlib.pyplot as plt


class Plots():

    def __init__(self, size, buffer_len, units, margin, axis):
        """Constructor:

        """
        self.data = numpy.zeros((6, buffer_len), numpy.float32)
        self.time = numpy.linspace(-buffer_len, -1, buffer_len)
        self.size = size
        self.axis = [
            [0, axis["height"]+margin],
            [0, axis["height"]+margin],
            [0, axis["height"]+margin],
            [0, axis["height"]+margin],
            [-2*axis["height"], 2*axis["height"]],
            [0, 1.1*axis["max_speed"]]]
        self.texts = [
            "Actuator 1 ("+units+")",
            "Actuator 2 ("+units+")",
            "Actuator 3 ("+units+")",
            "Actuator 4 ("+units+")",
            "Inclination ("+units+")",
            "Velocity ("+units+"/s)"
            ]

    def save_data(self, values, counter):
        position = counter % self.data.shape[1]
        values = numpy.reshape(values, (6,))
        self.data[:, position] = values
        self.time[position] = counter
        time_axis = [counter-len(self.time)+1, counter]
        figs = []
#         print("-----------------------------------------------------------")
#         print(numpy.hstack((self.time[counter+1:], self.time[:counter+1])))
        for signal, text, axis in zip(self.data, self.texts, self.axis):
            plt.figure(figsize=(self.size[0]/100, self.size[1]/100), dpi=100)
            plt.plot(
                numpy.hstack(
                    (self.time[position+1:], self.time[:position+1])),
                numpy.hstack(
                    (signal[position+1:], signal[:position+1])), 'b')
            plt.title(text)
            plt.axis(time_axis + axis)
            plt.grid(True)
            buf = io.BytesIO()
            plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
            plt.savefig(buf, format="png")
            plt.close()
            buf.seek(0)
            img_arr = numpy.frombuffer(buf.getvalue(), dtype=numpy.uint8)
            buf.close()
            img = cv2.imdecode(img_arr, 1)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            figs.append(img)
        return figs
