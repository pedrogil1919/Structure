'''
Created on 15 feb. 2021

@author: pedro.gil@uah.es

Class to return the list of distance errors when checking the position of the
structure.
'''


def greatest(v1, v2):
    return v1 if abs(v1) > abs(v2) else v2

def smallest(v1, v2):
    return v1 if abs(v1) < abs(v2) else v2

def merge_collision(res1, res2):
    
        if res1 and res2:
            return CollisionErrors()
        elif not res1 and res2:
            return res1
        elif res1 and not res2:
            return res2
        else:
            horizontal = greatest(res1.horizontal, res2.horizontal)
            actuator   = greatest(res1.actuator,   res2.actuator)
            central    = greatest(res1.central,    res2.central)
            front      = greatest(res1.front,      res2.front)
            rear       = greatest(res1.rear,       res2.rear)
            return CollisionErrors(
                False, horizontal, actuator, central, front, rear)

def merge_stability(res1, res2):
        if res1 and res2:
            return StabilityErrors()
        elif not res1 and res2:
            return res1
        elif res1 and not res2:
            return res2
        else:
            horizontal = greatest(res1.horizontal, res2.horizontal)
            return StabilityErrors(False, horizontal)

###############################################################################
###############################################################################

                
class CollisionErrors():
    '''
    classdocs
    '''


    def __init__(self, correct=True, horizontal=0.0, 
                 actuator=0.0, central=0.0, front=0.0, rear=0.0):
        '''
        Constructor
        
        Parameters:
        horizontal -- Horizontal distance needed for the wheel which is inside
          a step to place it outside the step, or distance needed to place the
          wheel in a stable position.
        actuator -- Vertical shift needed for an actuator to place the actuator
          in a valid position.
        central -- Vertical distance the structure need to move to place both
          the wheels and the actuators in a valid position.
        front -- Vertical distance the front actuator need to be shift to place
          the structure in a valid position.
        rear -- Same as front, for the rear actuator.
        '''
        self.correct    = correct
        self.horizontal = horizontal
        self.actuator   = actuator
        self.central    = central
        self. front     = front
        self.rear       = rear
        
    def __bool__(self):
        return self.correct
    
    def add_inclination_errors(self, inclination_errors):
        self.front = inclination_errors[0]
        self.rear = inclination_errors[1]
    
    def add_stability(self, error):
        
        if self and not error:
            self.horizontal = error.horizontal
            self.correct = False
        elif not self and not error:
            self.horizontal = greatest(self.horizontal, error.horizontal)
        

###############################################################################
###############################################################################

class StabilityErrors():
    
    def __init__(self, correct=True, front=0.0, rear=0.0):
        
        self.correct = correct
        if front is None and rear is None:
            self.horizontal = 0.0
        elif front is None and rear is not None:
            self.horizontal = rear
        elif front is not None and rear is None:
            self.horizontal = front
        else:
            self.horizontal = smallest(front, rear)

    def __bool__(self):
        return self.correct
    