'''
Created on 25 nov. 2021

@author: pedrogil

Class designed to compute the velocity profile for motions with a finite
acceleration and decceleration (which can be different).

'''

from enum import Enum

from math import sqrt
from numpy import arange, ndarray, hstack
from numpy import float64 as data_type
import matplotlib.pyplot as plt


class MinDistanceError(ValueError):
    @classmethod
    def description(cls):
        return "Can not reach end velocity wtihin the distance given. \
Enlarge distace or reduce gap between previous and end velocities."


class MaxVelocityError(ValueError):
    @classmethod
    def description(cls):
        return "The profile overpasses the maximum speed."


class MaxAccelerationError(ValueError):
    @classmethod
    def description(cls):
        return "The profile overpasses he maximum acceleration \
or decceleration."


class CeroVelocityError(ValueError):
    @classmethod
    def description(cls):
        return "The profile gets a negative velocity."


class ProfileClass(Enum):
    NoDynamics = 1
    ThreeSections = 2
    TwoSections = 3


class SpeedProfile():

    def __init__(self, dynamics_data, init_speed=0.0):

        self.prev_speed = init_speed
        self.speed = dynamics_data['speed']
        try:
            self.acceleration = dynamics_data['acceleration']
            self.decceleration = dynamics_data['decceleration']
        except KeyError:
            self.profile_type = ProfileClass.NoDynamics
        else:
            self.profile_type = ProfileClass.ThreeSections
        pass

    def compute_time(self, distance, next_speed):
        """Compute the minimum time required to complete the motion.

        """
        if self.profile_type == ProfileClass.NoDynamics:
            return distance / self.speed
        elif self.profile_type == ProfileClass.ThreeSections:
            return 0.0

    def profile_three_sections(self, v_end, d_tot, t_tot):
        """Compute acceleration and intermediate time for 3 section profile.

        This profile is used when the 2-sections profile reaches the object
        maximum velocity, wo we have to limit it.

        """
        # Compute mean velocity.
        v_max = self.speed
        # Normalize problem substracting initial velocity to all velocities.
        v_max -= self.prev_speed
        v_end -= self.prev_speed
        d_tot -= self.prev_speed * t_tot
        k = self.decceleration / self.acceleration

        k1 = (v_max - v_end) / v_max
        k2 = ((v_max + v_end) * t_tot - 2 * d_tot) / v_max

        t2 = 2.0

        t1 = k1 * t2 + k2

        a0 = v_max / t1
        a1 = -k * a0
        # Compute final tuplas:
        # Acceleration pairs.
        a = (a0, 0, a1)
        # Time pairs (remember that t1 + t2 = t_total
        t = (t2, t1 - t2, t_tot - t1)
        # Speeds pairs. Denormalize adding the initial speed to both values.
        v = (self.speed, v_end)
        return a, t, v

    def profile_two_sections(self, v_end, d_tot, t_tot):
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
            raise MaxVelocityError

        # Normalize problem substracting initial velocity to all velocities.
        v_mean -= self.prev_speed
        v_end -= self.prev_speed
        # Check type of profile. The limit happens when the mean velocity is
        # half of the final velocity once normilized, that is, substracted the
        # initial velocity to both velocities.
        v_mean2 = 2 * v_mean
        # When the mean velocity is the average value between the initial and
        # final velocity, this insttruction can be completed with only one
        # section (single acceleration if final velocity is greater than
        # initial velocity, o single decceleration otherwise).
        if v_mean2 == v_end:
            a1 = v_end / t_tot
            return (a1,), (t_tot,), (v_end + self.prev_speed,)
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
        # Check whether any limit has been reached.
        # Check acceleration limits:
        if a1 > 0:
            # Profile: Accelerate - Deccelerate:
            if a1 > self.acceleration or -a2 > self.decceleration:
                raise MaxAccelerationError
        else:
            if a2 > self.acceleration or -a1 > self.decceleration:
                raise MaxAccelerationError
        # Check velocity limits:
        if v1 > self.speed:
            raise MaxVelocityError
        elif v1 < 0.0:
            raise CeroVelocityError

        # Compute final tuplas:
        # Acceleration pairs.
        a = (a1, a2)
        # Time pairs (remember that t1 + t2 = t_total
        t = (t1, t2)
        # Speeds pairs. Denormalize adding the initial speed to both values.
        v = (v1 + self.prev_speed, v_end + self.prev_speed)
        return a, t, v

    def time_limit(self, a1, a2, v0, v2, d):

        # Compute intermediate variables:
        t0 = (v2 - v0) / a1
        k0 = a2 / a1
        # Compute second order equation terms:
        b2 = a1 * k0**2 + a2
        b1 = 2 * (v0 * k0 + a1 * t0 * k0 + v2)
        b0 = a1 * t0**2 + 2 * v0 * t0 - 2 * d

        # Solve second order equation to get the acceleration (see papers).
        # Get the discriminant:
        d = sqrt(b1**2 - 4 * b2 * b0)
        t2 = (-b1 + d) / (2 * b2)
        t1 = t0 + k0 * t2

        return t1 + t2

    def two_sections_time_limits(self, v_end, d_tot):
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
        if d_tot < self.min_distance(v_end):
            # If not possible, raise an error. The only solution to this
            # error is to modify accordingly the end velocity (or the previous
            # one if computing offline).
            raise MinDistanceError
        # Rename internal variables:
        v0 = self.prev_speed
        v2 = v_end
        # Profile: Accelerate - Deccelerate.
        # Rename internal variables:
        a1 = self.acceleration
        a2 = self.decceleration

        t_min = self.time_limit(a1, a2, v0, v2, d_tot)
        t_max = self.time_limit(-a2, -a1, v0, v2, d_tot)
        return t_min, t_max

    def min_distance(self, v_end):
        """Auxiliary function to compute the minimun distance.

        For the current previous velocity and the final velocity given, and
        using the current acceleration, this function computes the minimum
        distance the object can travel if the motion consist only in one
        section, according to its initial and the given end velocity
        (see docs).

        Returns the computed distance.

        """
        v_ini = self.prev_speed
        # Check if the motion should be accelerated or deccelerated, since
        # if the values are different, the time requirede in each case will
        # be different.
        if v_end > v_ini:
            d = (v_ini + v_end) * (v_end - v_ini) / (2 * self.acceleration)
        else:
            d = (v_ini + v_end) * (v_ini - v_end) / (2 * self.decceleration)
        return d

    def plot_dynamics(self, init_speed, accelerations, times, sample_time,
                      draw=True, block=True):
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
            plt.show(block=block)

        # And return all the signals computed.
        return time_list, speed_list, position_list

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
    # def compute_one_section(self, v_ini, v_end, d_tot, t_tot):
    #     """Compute acceleration and intermediate time for 1 section profile.
    #
    #     """
    #     v_aux = v_end - v_ini
    #     d_aux = d_tot - v_ini * t_tot
    #     t1 = 2 * (v_aux * t_tot - d_aux) / v_aux
    #     a1 = v_aux / t1
    #     t2 = t_tot - t1
    #     return (a1, 0), (t1, t2), (v_end, v_end)
