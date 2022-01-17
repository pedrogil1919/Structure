'''
Created on 25 nov. 2021

@author: pedrogil

Class designed to compute the speed profile for motions with a finite
acceleration and decceleration (which can be different).

Initial data are the initial and end speed for the motion, distance to travel
and total time to complete the motion.

The class computes a number of pairs acceleration time to accomplish with
the requirements. However, in some cases the motion can not be completed.

'''

# from enum import Enum

from math import sqrt, fabs
from numpy import arange, array, hstack
from numpy import float64 as data_type
import matplotlib.pyplot as plt

# If we want to use limit values, an exception can be raised because of the
# rounding errors. Use this value to perform the comparisons safely.
ROUND_ERROR = 1e-6


# This exception is raised when the pair of initial and end speed can not
# allow the objet to travel the required distance. In this case, the solution
# is to modify the initial and / or end speed. Use init_speed_range and
# end_speed_range to check for the range of valid speed.
class MinDistanceError(ValueError):
    @classmethod
    def description(cls):
        return "Can not reach end velocity wtihin the distance given. \
Enlarge distace or reduce gap between previous and end velocities."


# This exception is raised when the computed acceleration for one of the
# section is greater than the maximum acceleration. Use the function
# two_sections_time_limits to check the range of time intervals valid for the
# required motion.
class MaxAccelerationError(ValueError):
    @classmethod
    def description(cls):
        return "The profile overpasses he maximum acceleration \
or decceleration rate."


class SpeedProfile():

    def __init__(self, dynamics_data):

        # Save system parameters:
        # Maximum speed:
        self.speed = dynamics_data['speed']
        # Maximum acceleration and decceleration rates. Both values must be
        # positive values.
        self.acceleration = dynamics_data['acceleration']
        self.decceleration = dynamics_data['decceleration']

    def end_speed_range(self, v_ini, d_tot):
        """Computes range of end speeds.

        For a initial speed and a total distance to travel, there is a range of
        speed at which the motion can be finished. This range is function of
        the maximum acceleration and decceleration values.

        The function returns a tupla with this range (v_end_min, v_end_max).

        """
        # Compute minimum speed:
        # Check if the motion can use the maximum decceleration.
        if v_ini**2 < 2 * d_tot * self.decceleration:
            v_end_min = 0.0
        else:
            v_end_min = sqrt(v_ini**2 - 2 * d_tot * self.decceleration)
        # Compute maximum speed:
        v_end_max = sqrt(v_ini**2 + 2 * d_tot * self.acceleration)
        # Check if the motion reaches the maximum speed:
        if v_end_max > self.speed:
            v_end_max = self.speed
        return (v_end_min, v_end_max)

    def init_speed_range(self, v_end, d_tot):
        """Similar than end_speed_range, for the initial speed.

        """
        # Compute minimum speed:
        # Check if the motion can use the maximum decceleration.
        if v_end**2 < 2 * d_tot * self.acceleration:
            v_ini_min = 0.0
        else:
            v_ini_min = sqrt(v_end**2 - 2 * d_tot * self.acceleration)
        # Compute maximum speed:
        v_ini_max = sqrt(v_end**2 + 2 * d_tot * self.decceleration)
        # Check if the motion reaches the maximum speed:
        if v_ini_max > self.speed:
            v_ini_max = self.speed
        return (v_ini_min, v_ini_max)

    def time_limit(self, a1, a2, v0, v2, d):
        """Auxiliary function to solve the 2 order equation for time limits.

        """
        # Compute intermediate variables:
        t0 = (v2 - v0) / a1
        k0 = a2 / a1
        # Compute second order equation terms:
        b2 = a1 * k0**2 + a2
        b1 = 2 * (v0 * k0 + a1 * t0 * k0 + v2)
        b0 = a1 * t0**2 + 2 * v0 * t0 - 2 * d

        # Solve second order equation to get the acceleration (see papers).
        # Get the discriminant:
        # try:
        d = b1**2 - 4 * b2 * b0
        if d < 0:
            # If the discriminant is smaller than 0, this means that the system
            # reach 0 speed, so that in this case the time can be infinite.
            return float('inf'), 0.0
        # Solve the second order equation.
        t2 = (-b1 + sqrt(d)) / (2 * b2)
        # And compute the initial time.
        t1 = t0 + k0 * t2

        return t1, t2

    def min_distance(self, v_ini, v_end):
        """Auxiliary function to compute the minimun distance.

        For the current previous velocity and the final velocity given, and
        using the current acceleration (or decceleration), this function
        computes the minimum distance the object can travel if the motion
        consist only in one section, according to its initial and the given end
        velocity (see docs).

        Returns the computed distance.

        """
        # Check if the motion should be accelerated or deccelerated, since
        # if the values are different, the time requirede in each case will
        # be different.
        if v_end > v_ini:
            d = (v_ini + v_end) * (v_end - v_ini) / (2 * self.acceleration)
        else:
            d = (v_ini + v_end) * (v_ini - v_end) / (2 * self.decceleration)
        return d

    def max_speed_minimum_time(self, v_ini, v_end, d_tot):
        """Auxiliary function for the computation of time limits.

        when computing the time limits, it can happen than, for an 
        acceleration - decceleration profile, the maximum speed is reached. In
        this case, the minimum time shall be greater than the original one,
        since for some time, the motion will have the maximum speed, and this
        delays the object. This function computes this alternative time.

        """
        # Remane variables.
        v_max = self.speed
        a1 = self.acceleration
        a2 = self.decceleration
        # Compute time for the accelerate and deccelerate sections.
        t1 = (v_max - v_ini) / a1
        t2 = (v_max - v_end) / a2
        # Compute distance traveled in each section.
        d1 = 0.5 * (v_max + v_ini) * t1
        d2 = 0.5 * (v_max + v_end) * t2
        # Compute the remaining time, that must be traveled at constant speed.
        d_12 = d_tot - (d1 + d2)
        t12 = d_12 / v_max
        # The total time is the sum of the threee previous times.
        return t1 + t2 + t12

    def two_sections_time_limits(self, v_ini, v_end, d_tot):
        """Compute the time limits required to complete the motion.

        The function computes the minimum time required to complete the
        distance given, according to its previous velocity and the end
        velocity, using the actual maximum acceleration - deccerelation values.
        The minimum time is accieved when using the maximum accelerations,
        so that the motion is possible to be completed using more time, but
        never using less time than the computed here.

        The function also computes the maximum time the motion can be completed
        in, according to the same values as above.

        TODO: Check why if the velocity becomes 0 o bellow, the computation of
        the maximum distance fails (time becomes complex).

        Returns the minimum and maximum times.

        """
        # Check if the distance to travel is large enough to accomplish with
        # the required end velocity.
        if d_tot < self.min_distance(v_ini, v_end) - ROUND_ERROR:
            # If not possible, raise an error. The only solution to this
            # error is to modify accordingly the end velocity (or the initial
            # one if computing offline).
            raise MinDistanceError
        # Rename internal variables:
        v0 = v_ini
        v2 = v_end
        # Profile: Accelerate - Deccelerate.
        # Rename internal variables:
        a1 = self.acceleration
        a2 = self.decceleration

        # Compute the maximum time.
        t1_max, t2_max = self.time_limit(-a2, -a1, v0, v2, d_tot)
        t_max = t1_max + t2_max
        # Compute the minimum time to complete the motion.
        t1_min, t2_min = self.time_limit(a1, a2, v0, v2, d_tot)
        # For the minimum time, since the intermediate speed is greater than
        # the initial speed, it can also exceed the maximum speed.
        v1 = v0 + t1_min * a1
        if v1 > self.speed:
            t_min = self.max_speed_minimum_time(v_ini, v_end, d_tot)
        else:
            t_min = t1_min + t2_min
        return (t_min, t_max)

    ###########################################################################
    ###########################################################################
    ###########################################################################

    def compute_profile(self, v_ini, v_end, d_tot, t_tot):
        """Compute the speed profile to complete the motion.

        Returns the list of acceleration - time for the profile.

        Raise an error if the motion can not be completed.

        """
        # Check values:
        if v_ini < 0 or v_end < 0:
            raise ValueError("Speeds must be greater than 0")
        if v_ini > self.speed or v_end > self.speed:
            raise ValueError("Speeds must be smaller than maximum speed")
        an, tn, vn = self.profile_two_sections(v_ini, v_end, d_tot, t_tot)

        # Check acceleration limits:
        # NOTE: If we call this function with a time within the range returned
        # by the funcion two_sections_time_limits, there should not be any
        # error in the acceleration.
        for a in an:
            if a > 0:
                # Section with acceleration. Check maximum acceleration:
                if a > self.acceleration + ROUND_ERROR:
                    raise MaxAccelerationError
            else:
                if -a > self.decceleration + ROUND_ERROR:
                    raise MaxAccelerationError

        return an, tn, vn

    def profile_three_sections_max(self, v_ini, v_end, d_tot, t_tot):
        """Compute acceleration and intermediate time for 3 section profile.

        This profile is used when the 2-sections profile reaches the object
        maximum velocity, and we have to limit it.

        """
        # Compute mean velocity.
        v_max = self.speed
        # Normalize problem substracting initial velocity to all velocities.
        v_max -= v_ini
        v_end -= v_ini
        d_tot -= v_ini * t_tot
        k = self.decceleration / self.acceleration

        k1 = (v_max - v_end) / v_max
        k2 = ((v_max + v_end) * t_tot - 2 * d_tot) / v_max
        k3 = - k * v_max / (v_max - v_end)
        k4 = -k3 * t_tot

        t2 = (k4 - k2) / (k1 - k3)

        t1 = k1 * t2 + k2

        a0 = v_max / t1
        a1 = -k * a0
        # Compute final tuplas:
        # Acceleration.
        a = (a0, 0, a1)
        # Time (remember that t1 + t2 = t_total
        t = (t1, t2 - t1, t_tot - t2)
        # Speeds.
        v = (self.speed, v_end)
        return a, t, v

    def profile_three_sections_zero(self, v_ini, v_end, d_tot, t_tot):
        """Compute acceleration and intermediate time for 3 section profile.

        This profile is used when the 2-sections profile reaches zero velocity.
        Although this motion can steel be done with the two section profile,
        this means that the object move backwards in some moment. In this case
        it is better to stop the object when it reaches zero velocity, and
        wait there until the required time ellapses.

        """
        k = self.decceleration / self.acceleration
        v0 = v_ini
        v2 = v_end
        d = d_tot
        # Compute time intervals.
        t0 = 2 * d * v0 / (k * v2**2 + v0**2)
        t2 = (k * v2 + v0) * t0 / v0 - t0
        t1 = t_tot - t0 - t2
        # Compute accelerations:
        a2 = v0 / (k * t0)
        a1 = -k * a2
        # Compute final tuplas:
        # Accelerations:
        a = (a1, 0, a2)
        # Time:
        t = (t0, t1, t2)
        # Speeds:
        v = (0, v_end)
        return a, t, v

    def profile_one_section(self, v_ini, v_end, d_tot, t_tot):
        """Compute acceleration and intermediate time for 1 section profile.

        """
        v_aux = v_end - v_ini
        d_aux = d_tot - v_ini * t_tot
        t1 = 2 * (v_aux * t_tot - d_aux) / v_aux
        a1 = v_aux / t1
        t2 = t_tot - t1
        return (a1, 0), (t1, t2), (v_end, v_end)

    def profile_two_sections(self, v_ini, v_end, d_tot, t_tot):
        """Compute acceleration and intermediate time for 2 section profile.

        Arguments:
        v_end - required velocity at the end of the instruction.
        d_tot - Total distance to travel.
        t_tot - Time required to complete the distance.

        Returns:
        array with the list of acceleration, one for each section.
        array with the list of times, one foe each section.
        maximum (or minimum) velocity reached.

        Both arrays will have dimension 2 (since we are in 2 sections), althogh
        eventually can have only one, for the case when the motion is only
        accelerated (or deccelerated).
        """
        # Compute mean velocity.
        v_mean = d_tot / t_tot
        # Just to prevent mutiple computations, if the mean velocity required
        # is larger than the object maximum velocity, raise an error to extend
        # the time to complete the motion.
        if v_mean > self.speed:
            raise ValueError("Require more time.")

        # Normalize problem substracting initial velocity to all velocities.
        v_mean -= v_ini
        v_end -= v_ini
        # Check type of profile. The limit happens when the mean velocity is
        # half of the final velocity once normilized, that is, substracted the
        # initial velocity to both velocities.
        v_mean2 = 2 * v_mean
        # When the mean velocity is the average value between the initial and
        # final velocity, this insttruction can be completed with only one
        # section (single acceleration if final velocity is greater than
        # initial velocity, o single decceleration otherwise).
        if fabs(v_mean2 - v_end) < ROUND_ERROR:
            a1 = v_end / t_tot
            return (a1,), (t_tot,), (v_end + v_ini,)
        elif v_mean2 > v_end:
            # Since there are infinite solutions,we impose the restriction that
            # the current ratio between the deceleration and the acceleration
            # be equal to the ratio of the maximum of both magnitudes.
            k = self.decceleration / self.acceleration
        else:
            # In this case, the profile is the inverse of the original, that
            # is, first deccelarete, and the accelerate. The only difference is
            # that the first section must be done with the decceleration value,
            # and the second section with the acceleration. To solve this, it
            # is enough to change the value of k.
            k = self.acceleration / self.decceleration

        # Compute intermediate terms (see paper).
        a0 = v_end / t_tot
        v0 = 2 * v_mean - v_end
        t2 = t_tot
        v2 = v_end
        # Compute second order equation terms.
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
        t2 = t_tot - t1
        # Denormalize velocities:
        v1 += v_ini
        v_end += v_ini
        # # Check velocity limits:
        if v1 > self.speed:
            # If the system reaches the maximum velocity, switch the
            # profile to 3 sections one.
            # However, if the initial speed is close to the maximum speed, the
            # expresion for the computation of the 3 section profile raises an
            # error (zero division). In this case, use the alternative profile
            # of one section (one acceleration section and other at constant
            # speed).
            if fabs(v_end - self.speed) < 1e-2:
                # When is the end speed the one that is close to the maximum
                # use the implemented function.
                return self.profile_one_section(v_ini, v_end, d_tot, t_tot)
            if fabs(v_ini - self.speed) < 1e-2:
                # However, if the speed close to the maximum is the initial,
                # it is enough to "invert" time, and change the order of the
                # result.
                a, t, v = self.profile_one_section(v_end, v_ini, d_tot, t_tot)
                # In this case, change the order of the arrays, but also change
                # sign of the acceleration.
                a = (a[1], -a[0])
                return a, t[::-1], v[::-1]
            return self.profile_three_sections_max(v_ini, v_end, d_tot, t_tot)
        elif v1 < 0.0:
            return self.profile_three_sections_zero(v_ini, v_end, d_tot, t_tot)

        # Compute final tuplas:
        # Acceleration pairs.
        a = (a1, a2)
        # Time pairs (remember that t1 + t2 = t_total
        t = (t1, t2)
        # Speeds pairs. Denormalize adding the initial speed to both values.
        v = (v1, v_end)
        return a, t, v

    def plot_dynamics(self, init_speed, accelerations, times, sample_time):
        """Plot speed and position for a list of acceleration -time pairs.

        Arguments:
        initial_speed
        accelerations -- list of acceleration values.
        times -- list of time intervals. Must be the same size of acceleration
            list.
        sample_time -- need not be multiple of time intervals.
        """

        # List of arrrays to store all the samples computed.
        speed_list = array((init_speed,), data_type)
        position_list = array((0.0,), data_type)
        # Indices to get track of the time intervals and the samples for a
        # given section of the motion.
        current_sample = 0
        current_time = 0.0
        # Variables to compute the position and speed at the end of the section
        # independently of the sample time.
        current_position = 0.0
        current_speed = init_speed

        for acceleration, interval in zip(accelerations, times):

            # Compute the number of samples for the current section. Note that
            # we have to do this computation to ensure that we get the same
            # number of samples independently of the sample time being multiple
            # or not of the time intervals.

            # Get the time limits for the current section.
            last_time = current_time
            current_time += interval
            # Get the sample limits for the current section.
            last_sample = current_sample
            current_sample = int(current_time / sample_time)
            new_samples = current_sample - last_sample

            # When the sample time is not a multiple or the time intervals, we
            # have to take account of the remains of each side of the sample.
            # The time offset is the difference between the current time,
            # computed as the accumulation of the actual intervals, and the
            # position of the last sample, which is a multiple of the sample
            # time.
            time_offset = last_time - last_sample * sample_time
            # Get an auxiliary array for the computation of the intermediate
            # samples of the secion.
            samples = arange(1, new_samples + 1, 1, data_type)
            samples *= sample_time
            samples -= time_offset

            # Compute the intermediate values for the speed and position for
            # the current section.
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
            # different to the last sample, if the sample time is not multiple
            # of the time interval.
            current_position += \
                current_speed * interval + 0.5 * acceleration * interval**2
            # And this is the speed at the end of the section. Note that this
            # must be computed after the computation of the position.
            current_speed += acceleration * interval

        # One computed all the sections, generate a time array.
        time_list = arange(0, speed_list.shape[0], 1, data_type)
        time_list *= sample_time
        # time_list = arange(0, current_sample * sample_time,
        #                    sample_time, data_type)
        # time_list += sample_time

        # And return all the signals computed.
        return time_list, speed_list, position_list

    def draw_dynamics(self, time_data, speed_data, position_data,
                      max_time=None, block=False):
        # Plot figures.

        if max_time is None:
            max_time = time_data[-1]
        plt.subplot(2, 1, 1)
        plt.plot(time_data, speed_data, 'b')
        plt.title("velocidad")
        plt.grid(True)
        plt.xlim([0, max_time])

        plt.subplot(2, 1, 2)
        plt.plot(time_data, position_data, 'b')
        plt.title("espacio")
        plt.grid(True)
        plt.xlim([0, max_time])
        plt.show(block=block)

    def clear_figures(self):
        plt.figure(figsize=(8, 6), dpi=100)
    ###########################################################################

    # def compute_three_sections(self, v_ini, v_end, d_tot, t_tot):
    #     """Compute acceleration and intermediate time for 3 section profile.
    #
    #     """
    #     # Normalize problem substracting initial velocity to all velocities.
    #     v_end -= v_ini
    #     d_tot -= v_ini * t_tot
    #     # Compute second order equation terms.
    #     b2 = -(MAX_ACEL / MAX_DECL) * (MAX_ACEL + MAX_DECL)
    #     b1 = 2 * MAX_ACEL * (t_tot + v_end / MAX_DECL)
    #     b0 = - v_end**2 / MAX_DECL + 2 * d_tot
    #
    #     # Solve second order equation to get the acceleration (see papers).
    #     # Get the discriminant:
    #     d = sqrt(b1**2 - 4 * b2 * b0)
    #
    #     t1 = (-b1 + d) / (2 * b2)
    #     t2 = v_end / MAX_DECL - MAX_ACEL / MAX_DECL * t1 + t_tot
    #     # tn = (-b1 - d) / (2 * b2)
    #     v1 = MAX_ACEL * t1 - MAX_DECL * (t_tot - t2)
    #     return (-MAX_ACEL, 0, MAX_DECL), (t1, t2 - t1, t_tot - t2), (v1, v1, v_end)
    #
    #
    # def compute_time(self, distance, next_speed):
    #     """Compute the minimum time required to complete the motion.
    #
    #     """
    #     if self.profile_type == ProfileClass.NoDynamics:
    #         return distance / self.speed
    #     elif self.profile_type == ProfileClass.ThreeSections:
    #         return 0.0
