import math
import pomdp_py


class State(pomdp_py.State):
    def __init__(self, position,
                 goal=False, landmark=False, danger_zone=False, terminal=False):
        """
        position (tuple): (x,y) position of the agent on the grid.
        goal (bool): The agent is in a goal state.
        landmark (bool): The agent is in a landmark state.

        x axis is horizontal. y axis is vertical.
        """
        if not isinstance(position, tuple):
            raise ValueError("Expected tuple <position>")
        if not isinstance(goal, bool):
            raise ValueError("Expected boolean <goal>")
        if not isinstance(landmark, bool):
            raise ValueError("Expected boolean <landmark>")
        if not isinstance(danger_zone, bool):
            raise ValueError("Expected boolean <danger_zone>")
        self.position = position
        self.goal = goal
        self.landmark = landmark
        self.danger_zone = danger_zone
        self.terminal = terminal

    def __hash__(self):
        return hash((self.position,
                     self.goal,
                     self.landmark,
                     self.danger_zone,
                     self.terminal))

    def __eq__(self, other):
        if isinstance(other, State):
            return self.position == other.position \
                and self.goal == other.goal \
                and self.landmark == other.landmark \
                and self.danger_zone == other.danger_zone \
                and self.terminal == other.terminal
        else:
            return False

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "State(%s | Goal: %s | Landmark: %s | Danger: %s | Terminal: %s)" % \
            (str(self.position),
             str(self.goal)[0],
             str(self.landmark)[0],
             str(self.danger_zone)[0],
             str(self.terminal)[0])


class Action(pomdp_py.Action):
    def __init__(self, name):
        self.name = name

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        if isinstance(other, Action):
            return self.name == other.name
        elif type(other) == str:
            return self.name == other

    def __str__(self):
        return self.name

    def __repr__(self):
        return "Action(%s)" % self.name


class MoveAction(Action):
    NORTH = (0, -1)  # x is horizontal; x+ is right. y is vertical; y- is up.
    EAST = (1, 0)
    SOUTH = (0, 1)
    WEST = (-1, 0)

    def __init__(self, motion, name):
        if motion not in {MoveAction.NORTH, MoveAction.EAST,
                          MoveAction.SOUTH, MoveAction.WEST}:
            raise ValueError("Invalid move motion %s" % motion)
        self.motion = motion
        super().__init__("Move-%s" % str(name))


MoveNorth = MoveAction(MoveAction.NORTH, "NORTH")
MoveEast = MoveAction(MoveAction.EAST, "EAST")
MoveSouth = MoveAction(MoveAction.SOUTH, "SOUTH")
MoveWest = MoveAction(MoveAction.WEST, "WEST")


class Observation(pomdp_py.Observation):

    def __init__(self, position_reading=(math.inf, math.inf)):
        """
        position_reading: an (x,y) co-ordinate reading

        A position_reading of (0,0) means reading was returned.
        """

        if not isinstance(position_reading, tuple) and len(position_reading) != 2:
            raise ValueError("Expected position_reading to be a tuple pair.")

        self.position_reading = position_reading

    def __hash__(self):
        return hash(self.__str__())

    def __eq__(self, other):
        if isinstance(other, Observation):
            return self.position_reading == other.position_reading
        else:
            return False

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "Observation(%s)" % (
            "None" if self.position_reading == (math.inf, math.inf)
            else str(self.position_reading)
        )
