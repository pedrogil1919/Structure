"""
Created on 31 may. 2021

@author: pedro.gil@uah.es

Plotting variables as a function of sample_index using matplotlib.

This class plot six signals, the signals corresponding to the value of these
variables, in this order:
0 - Position of Actuator 1
1 - Position of Actuator 2
2 - Position of Actuator 3
3 - Position of Actuator 4
4 - Inclination of the structure.
5 - Structure velocity.

"""

import io
import numpy
import cv2
import matplotlib.pyplot as plt


class Plots():

    def __init__(self, size, buffer_len, units, axis):
        """Constructor:

        Arguments:
        size -- plot resolution (width x height) pixels.
        buffer_len -- Number of samples to plot in each render.
        units -- String with the name of the dimension units, for printing,
            for instance, cm, mm, m, ...
        axis -- axis limits for the plots.
        """
        # Create a 6-dimensional array to store the signal samples (create here
        # for not to create the same array each time we plot a new graphic9.
        self.data = numpy.zeros((6, buffer_len), numpy.float32)
        # Precomputed rray with the index of the sample to compute the time of
        # that sample.
        self.sample_index = numpy.linspace(-buffer_len, -1, buffer_len)
        self.size = size
        # Set the limits for the axis (this is the order):
        # - Actuator 1
        # - Actuator 2
        # - Actuator 3
        # - Actuator 4
        # - Inclination.
        # - Velocity.
        self.axis = [
            [0, axis["height"]],
            [0, axis["height"]],
            [0, axis["height"]],
            [0, axis["height"]],
            [-axis["height"], axis["height"]],
            [0, axis["max_speed"]]]
        self.texts = [
            "Actuator 1 (" + units + ")",
            "Actuator 2 (" + units + ")",
            "Actuator 3 (" + units + ")",
            "Actuator 4 (" + units + ")",
            "Inclination (" + units + ")",
            "Velocity (" + units + "/s)"
        ]

    def save_data(self, values, counter, sample_time):
        """Generate a plot for the six signals.

        Arguments:
        values -- Array of 6 elements, with the new values for the signals.
          The order must be the same as specified above.

        Return a list of six figures, each one is a plot of one of the 
        variables.

        """
        # Variable to index the time axis.
        # NOTE: This is only for not to create the same array in every
        # iteration.
        position = counter % self.data.shape[1]
        # Convert the array to a numpy array of dimension (6, 1).
        values = numpy.reshape(values, (6,))
        # And add the new incoming data to the signal buffer. Note that the
        # index "position" make a circular buffer, so that the oldest samples
        # are substituted by the new one.
        self.data[:, position] = values
        # Update time axis, so that the value represented in the horizontal
        # axis is the corect one (the value of the time at the rightmost
        # sample on the plots are the current tiem.
        self.sample_index[position] = counter
        # Set the axis limits for the time.
        time_axis = [sample_time * (counter - len(self.sample_index) + 1),
                     sample_time * counter]
        # And set the correct values for the time samples.
        time_signal = numpy.hstack(
            (sample_time * self.sample_index[position + 1:],
             sample_time * self.sample_index[:position + 1]))
        # Generate a list of figures, which are the figures to return.
        figs = []
        # print("-----------------------------------------------------------")
        # print(numpy.hstack((self.sample_index[counter+1:],
        # self.sample_index[:counter+1])))
        # For each signal, generate its corresponding figure.
        for signal, text, axis in zip(self.data, self.texts, self.axis):
            # Plot the figure. Do not forget to set:
            # matplotlib.use('Agg')
            # Otherwise, matplotlib can not generate plots that can be
            # converted to images.
            plt.figure(figsize=(self.size[0] / 100,
                                self.size[1] / 100), dpi=100)
            plt.plot(time_signal, numpy.hstack(
                    (signal[position + 1:], signal[:position + 1])), 'b')
            plt.title(text)
            plt.axis(time_axis + axis)
            plt.grid(True)
            plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
            # Convert the matplotlib plot to a opencv image (copied from the
            # matplotlib web).
            buf = io.BytesIO()
            plt.savefig(buf, format="png")
            plt.close()
            buf.seek(0)
            img_arr = numpy.frombuffer(buf.getvalue(), dtype=numpy.uint8)
            buf.close()
            img = cv2.imdecode(img_arr, 1)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            figs.append(img)
        return figs
