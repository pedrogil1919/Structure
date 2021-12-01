'''
Created on 25 nov. 2021

@author: pedrogil
'''

from math import sqrt, ceil
from numpy import arange, ndarray, hstack
from numpy import float64 as data_type
import matplotlib.pyplot as plt


def compute_simple_acceleration(v_ini, v_end, d_tot, t_tot, k):

    # Compute mean velocity.
    v_mean = d_tot / t_tot - v_ini
    v_end -= v_ini

    # Compute intermediate terms.
    a0 = v_end / t_tot
    v0 = 2 * v_mean - v_end
    t2 = t_tot
    v2 = v_end
    # Comtupe second order equation terms
    c = -a0 * v_end
    b = v2 - k * a0 * t2 - v0 * (1 + k)
    a = k * t2

    d = sqrt(b**2 - 4 * a * c)
    ap = (-b + d) / (2 * a)
    an = (-b - d) / (2 * a)

    tp = v0 / (ap - a0)
    tn = v0 / (an - a0)
    if tp > 0 and tp < t_tot:
        a1 = ap
        t1 = tp
    elif tn > 0 and tn < t_tot:
        a1 = an
        t1 = tn

    v1 = a1 * t1

    return a1, t1, v1


def plot_dynamics(init_speed, accelerations, times, sample_time, draw=False):
    """Plot speed and position for a list of acceleration -time pairs.

    Arguments:
    initial_speed
    accelerations -- list of acceleration values.
    times -- list of time intervals. Must be the same size of acceleration
        list.
    sample_time -- need not be multiple of time intervals.
    """

    # List of arrrays to store all the samples computed.
    speed_list = ndarray((0,), data_type)
    position_list = ndarray((0,), data_type)
    # Indices to get track of the time intervals and the samples for a given
    # section of the motion.
    current_sample = 0
    current_time = 0.0
    # Variables to compute the position and speed at the end of the section
    # independently of the sample time.
    current_position = 0.0
    current_speed = init_speed

    for acceleration, interval in zip(accelerations, times):

        # Compute the number of samples for the current section. Note that we
        # have to do this computation to ensure that we get the same number of
        # samples independently of the sample time being multiple or not of
        # the time intervals.

        # Get the time limits for the current section.
        last_time = current_time
        current_time += interval
        # Get the sample limits for the current section.
        last_sample = current_sample
        current_sample = int(current_time / sample_time)
        new_samples = current_sample - last_sample

        # When the sample time is not a multiple or the time intervals, we have
        # to take account of the remains of each side of the sample. The time
        # offset is the difference between the current time, computed as the
        # accumulation of the actual intervals, and the position of the last
        # sample, which is a multiple of the sample time.
        time_offset = last_time - last_sample * sample_time
        # Get an auxiliary array for the computation of the intermediate
        # samples of the secion.
        samples = arange(1, new_samples + 1, 1, data_type)
        samples *= sample_time
        samples -= time_offset

        # Compute the intermediate values for the speed and position for the
        # current section.
        # Compute samples for the speed.
        speed_samples = current_speed + acceleration * samples
        # And compute also the samples for the position traveled.
        position_samples = current_position + \
            current_speed * samples + 0.5 * acceleration * samples**2
        # Add new samples to the actual ones.
        speed_list = hstack((speed_list, speed_samples))
        position_list = hstack((position_list, position_samples))

        # Get the values for the end of the section. This make the function
        # independent of the sample time.
        # This is the position at the end of the section, that can be
        # different to the last sample, if the sample time is not multiple of
        # the time interval.
        current_position += \
            current_speed * interval + 0.5 * acceleration * interval**2
        # And this is the speed at the end of the section. Note that this must
        # be computed after the computation of the position.
        current_speed += acceleration * interval

    # One computed all the sections, generate a time array.
    time_list = arange(0, current_sample * sample_time, sample_time, data_type)
    time_list += sample_time

    # Plot figures, if required.
    if draw:
        # Plot figures.
        plt.figure(figsize=(8, 6), dpi=100)

        plt.subplot(2, 1, 1)
        plt.plot(time_list, speed_list, 'b')
        plt.title("velocidad")
        plt.grid()
        plt.xlim([0, current_time])

        plt.subplot(2, 1, 2)
        plt.plot(time_list, position_list, 'b')
        plt.title("espacio")
        plt.grid()
        plt.xlim([0, current_time])
        plt.show()

    # And return all the signals computed.
    return time_list, speed_list, position_list


def draw_dynamics2(initial_speed, accelerations, times, sample_time):
    # List of arrrays to store all the samples computed.
    time_list = ndarray((0,), data_type)
    speed_list = ndarray((0,), data_type)
    position_list = ndarray((0,), data_type)

    # Variables needed to compute the values at the end of the section. This
    # is needed for when the sample time is not multiple of the time intervals.
    last_time = 0.0
    last_speed = initial_speed
    last_position = 0.0
    # To allow the system to work with non-integer sample times, we have to
    # take account of the difference between the last sample and the ending
    # time, to add this time to the distance traveled for the next section.
    remaining_time = 0.0
    # This variable is needed to allow the function to compute samples at
    # uniform time, independently of the sample time.
    last_sample = 0

    for acceleration, total_time in zip(accelerations, times):
        if total_time < 0:
            raise ValueError("Time intervals must be positive.")
        #######################################################################
        # Now, compute the samples:
        current_time = last_sample * sample_time
        # Compute the number of samples for this section.
        total_samples = ceil(total_time / sample_time)
        # Generate the indices for all samples.
        samples = arange(0, total_samples, 1, data_type)
        # Generate the time, with respect to 0, for all samples. This way,
        # we generate uniform time samples, even if the sample time is not
        # multiple of all the time intervals.
        time_samples = sample_time * samples + remaining_time

        last_sample += total_samples

        # Compute samples for the speed.
        speed_samples = last_speed + acceleration * time_samples
        # And compute also the samples for the position traveled.
        position_samples = last_position + \
            last_speed * time_samples + \
            0.5 * acceleration * time_samples**2
        # Add new samples to the actual ones.
        time_list = hstack(
            (time_list, (time_samples + current_time - remaining_time)))
        speed_list = hstack((speed_list, speed_samples))
        position_list = hstack((position_list, position_samples))

        #######################################################################
        # Get the values for the end of the section. This make the function
        # independent of the sample time.
        # This is the position at the end of the section, that can be
        # different to the last sample, if the sample time is not multiple of
        # the time interval.
        last_position += \
            last_speed * total_time + 0.5 * acceleration * total_time**2
        last_time += total_time
        last_speed += acceleration * total_time

        remaining_time = last_time - (last_sample - 1) * sample_time

    for t, p in zip(time_list, position_list):
        print("%.2f, %.3f" % (t, p))
    # Plot figures.
    plt.figure(figsize=(8, 6), dpi=100)

    plt.subplot(2, 1, 1)
    plt.plot(time_list, speed_list, 'b')
    plt.title("velocidad")
    plt.grid()
    plt.xlim([0, last_time])

    plt.subplot(2, 1, 2)
    plt.plot(time_list, position_list, 'b')
    plt.title("espacio")
    plt.grid()
    plt.xlim([0, last_time])
    plt.show()

    return last_position
