'''
Created on 28 ene. 2021

@author: pedro.gil@uah.es

'''

from enum import Enum

# Position of a wheel with respect to the steps of the stairs (see
# wheelStates.svg).
#   - Air: Not touching any solid surface.
#   - Ground: Wheel lying on the ground.
#   - Contact: Wheel on contact with the vertical part of the step.
#   - Corner: Wheel on the inner corner of the step.
#   - Unstable: Wheel on the outer corner of the step (not desirable position).
#   - Outer: Wheel over the outer corner of the step.
#   - Over: On air, but higher than the right and left steps.
#   - Inside: Inside any part (not possible, just to detect collisions).
#   - Unchecked: The state of the wheel has not been checked yet.


class WheelState(Enum):
    Air = 1
    Ground = 2
    Contact = 3
    Corner = 4
    Unstable = 5
    Outer = 6
    Over = 7
    Inside = 8
    Uncheked = 9


MAX_GAP = 0.05
# Maximum gap allowed to consider an object touching the ground or the steps
# See maxGap.svg figure.

###############################################################################
# End of file.
###############################################################################