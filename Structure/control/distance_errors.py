'''
Created on 15 feb. 2021

@author: pedro.gil@uah.es

Class to return the list of distance errors when checking the position of the
structure.
'''

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
        self.correct = correct
        self.horizontal = horizontal
        self.actuator = actuator
        self.central = central
        self. front = front
        self.rear = rear
        
    def __bool__(self):
        return self.correct
        
class StabilityErrors():
    
    def __init__(self, correct=True, horizontal=0.0):
        
        self.correct = correct
        self.horizontal = horizontal

    def __bool__(self):
        return self.correct