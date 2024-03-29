"""
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to include functions related with graphics to display results on screen
or to save a video.

"""

import numpy
import cv2
import os

from graphics.plots import Plots


class Graphics:

    def __init__(self, img_data, video_data, csv_data, sample_data, axis=None):
        """Constructor:

        Arguments:
        image_data: dictionary with the following keys:
            size -- Size of the image (height, width)
            origin -- Vertical shift of the stairs with respect to the image
                top.
            scale -- Scale in units/pixel, where units : (cm, mm, m, inc, ...).
                If None, drawing functions computes the scale so that the whole
                list of stairs spreads over the whole width of the image.

        video_data: dictionary with the following keys:
            video_dir -- If a directory, save a video image sequence in this
                directory. If the directory does not exits, it is created. If
                it can not be created, raises the proper exception.
            display -- If True, display image on screen. program can be stopped
                by pressing Esc key. If False, program can not be finished.
            pause -- If an image is to be displayed on screen, if pause is True
                the program waits after each image until the user press Enter.
                If False, the video continues. This value can be changed on
                runtime pressing Space key.
            interval -- When pause is False, and display True, 1/framerate for
                the sequence displayed on screen.
            units -- String with the name of the dimension units, for instance,
                m, cm, mm, ...

        """
        self.sample_time = sample_data['sample_time']
        self.time_units = sample_data['time_units']
        self.prv_pos = None
        # Flag to switch between manual mode operation, or automatic.
        self.manual_mode = False
        # Create image.
        size = img_data['size']
        self.image = numpy.full((size[0], size[1], 3), 0xFF, numpy.uint8)
        # OpenCV antialiased line parameter (see OpenCV line() function)
        self.shift = 3      # Number of fractional bits.
        self.scale = img_data['scale']
        # Save image parameters.
        self.origin = img_data['shift']
        # Display image on screen after each iteration.
        self.display = video_data['display']
        if self.display:
            cv2.namedWindow("res")
            cv2.moveWindow("res", 0, 0)
            self.toggle_pause = video_data['pause']

            self.interval = video_data['interval']

        # Frame counter.
        # self.counter = 0
        if video_data['video_dir'] is not None:
            # If an image sequence need to be created:
            # Create directory:
            self.video_dir = video_data['video_dir']
            try:
                os.makedirs(self.video_dir)
            except FileExistsError:
                # If the directory already exits, do nothing.
                pass
            self.save_video = True
            # Check whether we also have to save the composited video, that
            # is, the image of the structure plus the state of the actuators.
            if video_data['composition'] is not None:
                self.dir_comp = os.path.join(
                    self.video_dir, video_data['composition'])
                try:
                    os.makedirs(self.dir_comp)
                except FileExistsError:
                    # If the directory already exits, do nothing.
                    pass
                self.save_composition = True
                # Generate the image where the program is going to compose
                # the image. The image must be doble in both dimensions (see
                # images layout).
                self.composite_image = numpy.full(
                    (2 * size[0], 2 * size[1], 3), 0xFF, numpy.uint8)
                # Variables to compute images offsets relative to the
                # composited image.
                # NOTE: If the height is even, one row of the strucuture
                # image is lost, but this is not unimportant. However, try to
                # image size to an odd number in both directions, since this
                # is the common.
                h = size[0] // 2
                w = size[1]
                # Build RoIs (OpenCV rectangular region of interest) where
                # each graph must be placed (see image layout).
                self.RoI = {
                    "video": (slice(0, size[0], 1), slice(0, size[1], 1)),
                    "ac_L1": (slice(1 * 0, 1 * h, 1), slice(1 * w, 2 * w, 1)),
                    "ac_L2": (slice(1 * h, 2 * h, 1), slice(1 * w, 2 * w, 1)),
                    "ac_L3": (slice(2 * h, 3 * h, 1), slice(1 * w, 2 * w, 1)),
                    "ac_L4": (slice(3 * h, 4 * h, 1), slice(1 * w, 2 * w, 1)),
                    "vel_0": (slice(2 * h, 3 * h, 1), slice(1 * 0, 1 * w, 1)),
                    "incli": (slice(3 * h, 4 * h, 1), slice(1 * 0, 1 * w, 1))
                }
                self.plots = Plots((w, h), video_data["buffer_size"],
                                   video_data["units"], axis)
            else:
                self.save_composition = False
        else:
            self.save_video = False

        # Configure files to save data to csv files.
        if csv_data['csv_dir'] is not None:
            csv_dir = csv_data['csv_dir']
            try:
                os.makedirs(csv_dir)
            except FileExistsError:
                # If the directory already exits, do nothing.
                pass
            self.save_csv = True
            self.csv_files = (
                open(os.path.join(csv_dir, csv_data['actuator1']), "w"),
                open(os.path.join(csv_dir, csv_data['actuator2']), "w"),
                open(os.path.join(csv_dir, csv_data['actuator3']), "w"),
                open(os.path.join(csv_dir, csv_data['actuator4']), "w"),
                open(os.path.join(csv_dir, csv_data['actuator9']), "w"),
                open(os.path.join(csv_dir, csv_data['speed_0']), "w"),
                open(os.path.join(csv_dir, csv_data['speed_1']), "w"),
                open(os.path.join(csv_dir, csv_data['speed_2']), "w"),
                open(os.path.join(csv_dir, csv_data['speed_3']), "w"))
        else:
            self.save_csv = False

    def set_manual_mode(self):
        """Set to manual mode, so the user can move the structure manually."""
        self.manual_mode = True
        if not self.display:
            # If we are not displaying images, the system can not be set to
            # manual mode, since the user can not interact with the system.
            # Raise an error to warm the calling function.
            raise ValueError

    def draw(self, stairs, structure, counter, pause=False):
        """Generate an image of the actual elements.

        If the object was configured with display set to True, the program can
        be paused with the Space key.
        If video was set to a directory, a video image sequence will be created
        in this directory.
        Return False is the user press the key to finish the program (escape).
        Return also the key pressed.

        Arguments:
        stairs, structure -- Elements to draw.
        pause -- If True, and display is True, the program pause until the user
            press a Key. If False, an internal variable check whether to pause
            or not.

        """
        # Clear the image to white.
        self.image[:] = 0xFF
        # If global scale is not given, compute the scale to fix the whole
        # system into the window.
        if self.scale is None:
            self.scale = self.image.shape[1] / stairs.length()
        # Antialiased scale (see OpenCV draw documentation).
        aa_scale = self.scale * (1 << self.shift)
        # Compute the vertical shift so that the system appears in the center
        # of the image.
        # Compute the total height (stairs plus structure). Take into account
        # that the height of the structure is not correct, since w should need
        # to add the wheel radius, but this is unimportant.
        # The parameter origin also allow the user to shift the drawing some
        # pixels from the center of the image.
        origin = (self.image.shape[0] + self.origin) / (2 * self.scale)
        total_height = structure.HEIGHT + stairs.height()
        origin += total_height / 2
        # Draw the stairs.
        stairs.draw((0, origin), self.image, aa_scale, self.shift)
        # Draw the structure.
        structure.draw((0, origin), self.image, aa_scale, self.shift)
        # structure.draw_wheel_trajectory(
        #     self.origin, self.image, aa_scale, self.shift, 3)
        # Draw OSD information.
        current_time = counter * self.sample_time
        print_time = "%8.2f %s" % (current_time, self.time_units)
        cv2.putText(self.image, print_time,
                    (20, self.image.shape[0] - 30), 1, 5, 0x00, 4)
        c = 0
        if self.display:
            # If in manual mode, draw a red rim around the image frame.
            if self.manual_mode:
                cv2.rectangle(self.image, (0, 0), self.image.shape[1::-1],
                              (0x00, 0x00, 0xFF), 4)
            while True:
                # Display image on screen:
                cv2.imshow("res", self.image)
                if pause or self.manual_mode:
                    wait_time = 0
                else:
                    wait_time = 0 if self.toggle_pause else self.interval
                # NOTE: Just in case it happens again. If you set the
                # matplotlib library to agg and use cv2.waitKey, if you press
                # a key other than enter on the opencv window, the program
                # crashes. So use agg only when you are not going to use
                # opencv key events.
                c = cv2.waitKey(int(wait_time)) & 0x7F
                ###############################################################
                if c == 27:
                    # Escape key.
                    return False, 0
                elif c == ord(' '):
                    # Space key.Toggle between pause and not pause.
                    self.toggle_pause = not self.toggle_pause
                    # Set key to an invalid code, to avoid conflict with
                    # manual control function.
                    c = 0xFF
                elif c == 9:  # Tab key
                    self.manual_mode = not self.manual_mode
                    # When leaving manual mode, start in pause mode. This
                    # instruction ensure that.
                    self.toggle_pause = True
                    # Set key to an invalid code, to avoid conflict with
                    # manual control function.
                    c = 0xFF
                elif c == 13:  # Enter key.
                    pass
                else:
                    # If we are in automatic mode and in pause mode, the only
                    # way to exit the function is by pressing the available
                    # keys:
                    # - Space: To switch to no pause mode.
                    # - Enter: To perform one next step of the instruction.
                    # - Tab: To switch to manual mode.
                    # - Esc: To finish the program.
                    if not self.manual_mode and self.toggle_pause:
                        continue
                break

        # Get current position of the actuators.
        values = structure.actuator_positions()
        wheel_pos = structure.wheel_positions()
        if self.prv_pos is None:
            self.prv_pos = wheel_pos

        # Get wheel speed.
        speed0 = (wheel_pos[0][0] - self.prv_pos[0][0]) / self.sample_time
        speed1 = (wheel_pos[1][0] - self.prv_pos[1][0]) / self.sample_time
        speed2 = (wheel_pos[2][0] - self.prv_pos[2][0]) / self.sample_time
        speed3 = (wheel_pos[3][0] - self.prv_pos[3][0]) / self.sample_time
        self.prv_pos = wheel_pos

        values.append(speed0)
        values.append(speed1)
        values.append(speed2)
        values.append(speed3)
        if self.save_csv:
            for f, v in zip(self.csv_files, values):
                f.write("%0.10f, %.10f\n" % (current_time, v))

        if self.save_video:
            # Save image in the image sequence directory.
            # Generate image name.
            aux_name = "image%05i.png" % counter
            image_name = os.path.join(self.video_dir, aux_name)
            cv2.imwrite(image_name, self.image)
            if self.save_composition:
                # Generate the rest of the graphics.
                figs = self.plots.save_data(
                    values[0:6], counter, self.sample_time)
                self.composite_image[self.RoI['video']] = self.image
                self.composite_image[self.RoI['ac_L1']] = figs[0]
                self.composite_image[self.RoI['ac_L2']] = figs[1]
                self.composite_image[self.RoI['ac_L3']] = figs[2]
                self.composite_image[self.RoI['ac_L4']] = figs[3]
                self.composite_image[self.RoI['vel_0']] = figs[4]
                self.composite_image[self.RoI['incli']] = figs[5]
                composite_name = os.path.join(self.dir_comp, aux_name)
                cv2.imwrite(composite_name, self.composite_image)
        return True, c

###############################################################################
# End of file.
###############################################################################
