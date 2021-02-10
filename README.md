# Stairs Climbing WheelChair Simulator Code

## Examples:

These videos shows the operation of the structure when crossing different types of stairs:

- [Simple upstairs](https://youtu.be/5JVE5Z-6W90)
- [Simple downstairs](https://youtu.be/L5EnwhP_i_0)
- [Doble upstairs](https://youtu.be/xWDOuWn6PaY)
- [Doble downstairs](https://youtu.be/J9GNdBwLiBI)
- [Up down](https://youtu.be/VLIcXlqMZjc)
- [Down up](https://youtu.be/QYRTae0HUY0)
- [Obstacles](https://youtu.be/dOSk67Nq8ec)
- [Different steps](https://youtu.be/A3KZjVMgG6M)

## Installation:

**Requirements:**

- Python 3 for running the code.
- OpenCV and Python-OpenCV for displaying and saving image sequences.

**Running the code:**

To run the code, open a terminal, and type:

```
$ python3 main_loop.py
```

The wheelchair structure dimensions and stairs size can be configured with a xml file. You can create your own xml file, or modify and reuse the file *settings.xml* provided. Please, read tag definition inside this file to understand parameter configuration. If you use a different xml file, or in a different directory, add this file when running the program:

```
$ python3 main_loop.py /path/to/file/settings.xml
```

When running the code, the program starts in automatic mode. In automatic mode, the program generates the instructions to cross the stairs, and run its way to the end of it. If the tag <graphics/framerate/pause> is set to False, the program will execute in a continuos fashion. If set to True, the program will perform one step by pressing the Enter Key. You can always switch between continuos and step by step modes by pressing Space key. Pressing Escape at any time will finish the program.


At any time, you can switch to manual mode by pressing Tab Key (in some occasions, you need to press Tab Key twice). When in manual mode a red rim surrounding the frame of the image will be displayed. In this mode, you can move the structure manually, by pressing one of the following keys:
- 8: Elevate structure.
- 2: Take down structure.
- 6: Move structure to the right.
- 4: Move structure to the left.
- q, (w, e, r): Shift actuator 1, (2, 3, 4) upwards.
- a, (s, d, f): Shift actuator 1, (2, 3, 4) downwards.
- t, (g): Incline the structure a postive, (negative) angle, raising the front actuator, and fixing the rear wheel.
- y, (h): Incline the structure a postive, (negative) angle, raising the front actuator, and fixing the front wheel.
- u, (j): Incline the structure a postive, (negative) angle, raising the rear actuator, and fixing the rear wheel.
- i, (k): Incline the structure a postive, (negative) angle, raising the rear actuator, and fixing the front wheel.

Pressing Tab Key again switch back to automatic mode. In this case, the program always returns in step by step mode. You can press Space key to switch to continuos mode.

If the xml file includes the tag video, a image sequence will be created in the directory given. You can postprocess this squence with a video editor to create a video (mpeg, avi, ...) file.

## Documentation:

**Class definition:**

The following figure shows the main classes defined, and their relationships:\
\
\
\
![Class structure](https://github.com/pedrogil1919/WheelChair/blob/master/wheelchair/docs/structure.svg)

- **Base**: defines the main dimensions of the structure, and the array of actuator (4). It also exposes the functions to move the different elements of the structure (actuators and wheels).
- **Actuator** (x4): It it used to join the ending wheel with the base structure. It exposes to the base the function to shift the actuator vertically.
- **Joint** (x4): Basically used for the trigonometric computations when the structure is inclined (compute height and distance of the upper ending of the actuator with respect to the structure origin of coordinates).
- **Wheel** (x4): It exposes the functions to check wheel state with respect to the stairs steps (detect collisions and/or position of the wheel with respect to the step, that is, in contact or not in contact).
- **Physics**: It stores the stairs definition, and performs the computation to detect wheel collisions, and/or wheel position with respect to the steps.
- **Control**: Two different blocks:
  - **Manual**: Check for user keyboard input and move the structure accordingly.
  - **Automatic**: Current implementation of the algorithm to cross a stair.
- **Graphics**: Function to display results on screen, or save image sequence to disk.
