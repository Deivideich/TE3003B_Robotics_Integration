from enum import Enum

class LifterState(Enum):
    MOVE_FORK_TO_BOTTOM = 0
    MOVE_FORK_TO_TOP = 1
    MOVE_FORK_TO_MIDDLE = 2
    LEAVE_PALLET = 3
    STOP = 4
    
class LifterStatus(Enum):
    STOP = 0
    MOVE_UP = 1
    MOVE_DOWN = 2
    
