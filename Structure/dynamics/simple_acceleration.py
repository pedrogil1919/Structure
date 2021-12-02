'''
Created on 25 nov. 2021

@author: pedrogil
'''

from math import sqrt
from numpy import arange, ndarray, hstack
from numpy import float64 as data_type
import matplotlib.pyplot as plt

MAX_ACEL = 2  # m/s^2


def compute_two_sections(v_ini, v_end, d_tot, t_tot, k):
    """Compute acceleration and intermediate time for 2 section profile.

    """
    # Compute mean velocity.
    v_mean = d_tot / t_tot
    # Normalize problem substracting initial velocity to all velocities.
    v_mean -= v_ini
    v_end -= v_ini
    # Check type of profile. The limit happens when the mean velocity is half
    # of the final velocity once normilized, that is, substracted the initial
    # velocity to both velocities.
    v_mean2 = 2 * v_mean
    # When the mean velocity is the average value between the initial and
    # final velocity, this insttruction can be completed with only one
    # section (single acceleration if final velocity is greater than
    # initial velocity, o single decceleration otherwise).
    if v_mean2 == v_end:
        a1 = v_end / t_tot
        print("1 section")
        return a1, t_tot, v_end
    elif v_mean2 < v_end:
        # In this case, the profile is the inverse of the original, that is,
        # first deccelarete, and the accelerate. The only difference is that
        # the first section must be done with the decceleration value, and the
        # second section with the acceleration. To solve this, it is enough to
        # change the value of k.
        k = 1 / k

    # Compute intermediate terms (see paper).
    a0 = v_end / t_tot
    v0 = 2 * v_mean - v_end
    t2 = t_tot
    v2 = v_end
    # Comtupe second order equation terms.
    b2 = k * t2
    b1 = v2 - k * a0 * t2 - v0 * (1 + k)
    b0 = -a0 * v_end

    # Solve second order equation to get the acceleration (see papers).
    # Get the discriminant:
    d = sqrt(b1**2 - 4 * b2 * b0)
    # The second order solution depends on the type of profile. If the mean
    # velocity if greater than half of the final velocity, the profile is
    # always first accelerate, and the solution is the one considering the
    # positive discriminant.
    if v_mean2 < v_end:
        # Positive solution.
        d = -d
    a1 = (-b1 + d) / (2 * b2)
    # Compute the rest of the variables:
    t1 = v0 / (a1 - a0)
    v1 = a1 * t1
    a2 = -a1 * k
    return a1, a2, t1, v1 + v_ini


def compute_one_section(v_ini, v_end, d_tot, t_tot, k):
    """Compute acceleration and intermediate time for 2 section profile.

    """

    v_aux = v_end - v_ini
    d_aux = d_tot - v_ini * t_tot
    t1 = 2 * (v_aux * t_tot - d_aux) / v_aux
    a1 = v_aux / t1
    return a1, 0, t1, v_end


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
