'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

Module to include functions related with graphics to display results on screen
or to save a video.

'''

import numpy
import cv2
import os


class Graphics:

    def __init__(self, image_data, video_data):
        """Constructor:

        Parameters:
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

        """
        # Flag to switch between manual mode operation, or automatic.
        self.manual_mode = False
        # Create image.
        size = image_data['size']
        self.image = numpy.full((size[0], size[1], 3), 0xFF, numpy.uint8)
        # OpenCV antialiased line parameter (see OpenCV line() function)
        self.shift = 3      # Number of fractional bits.
        self.scale = image_data['scale']
        # Save image parameters.
        self.origin = (0, image_data['shift'] / self.scale)
        # Display image on screen after each iteration.
        self.display = video_data['display']
        if self.display:
            cv2.namedWindow("res")
            cv2.moveWindow("res", 0, 0)
            self.toggle_pause = video_data['pause']

            self.interval = video_data['interval']
        # Frame counter.
        self.counter = 0
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
        else:
            self.save_video = False

    def draw(self, stairs, structure, pause=False):
        """Generates an image of the actual elements.

        If the object was configured with display set to True, the program can
        be paused with the Space key.
        If video was set to a directory, a video image sequence will be created
        in this directory.
        Returns False is the user press the key to finish the program (escape).
        Returns also the key pressed.

        parameters:
        stairs, structure -- Elements to draw.
        pause -- If True, and display is True, the program pause until the user
            press a Key. If False, an internal variable check whether to pause
            or not.
        """
        # Clear the image to white.
        self.image[:] = 0xFF
        # Antialiased scale (see OpenCV draw documentation).
        if self.scale is None:
            scale = self.image.shape[1] / stairs.length()
        else:
            scale = self.scale
        aa_scale = scale * (1 << self.shift)
        # Draw the stairs.
        stairs.draw(self.origin, self.image, aa_scale, self.shift)
        # Draw the structure.
        structure.draw(self.origin, self.image, aa_scale, self.shift)
#         structure.draw_wheel_trajectory(
#             self.origin, self.image, aa_scale, self.shift, 3)
        # Draw OSD information.
        cv2.putText(self.image, str(self.counter),
                    (20, self.image.shape[0]-30), 1, 5, 0x00, 4)
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
                c = cv2.waitKey(int(wait_time)) & 0x7F
                ###############################################################
                if c == 27:
                    # Escape key.
                    return False, self.manual_mode, 0
                elif c == ord(' '):
                    # Space key. Toggle between pause and not pause.
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
                    # Space: To switch to no pause mode.
                    # Enter: To perform one next step of the instruction.
                    # Tab: To switch to manual mode.
                    # Esc: To finish the program.
                    if not self.manual_mode and self.toggle_pause:
                        continue
                break
        if self.save_video:
            # Save image in the image sequence directory.
            # Generate image name.
            aux_name = "image%05i.png" % self.counter
            image_name = os.path.join(self.video_dir, aux_name)
            cv2.imwrite(image_name, self.image)
        self.counter += 1
        return True, self.manual_mode, c

###############################################################################
# End of file.
###############################################################################