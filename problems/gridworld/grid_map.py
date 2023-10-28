import copy
import math
import random


class GridMap:

    def __init__(self, n, m, obstacles, landmarks, danger_zones, goals):
        """
        n: number of rows (x-direction)
        m: number of columns (y-direction)
        obstacles: list of tuples indicating co-ordinates of obstacles.
        landmarks: list of tuples indicating co-ordinates of landmarks.
        goals: a list of tuples indicating the co-ordinates of the goal states.
        danger_zones: a list of tuples indicating the co-ordinates of danger zones.
        """
        self.n = n
        self.m = m
        self.obstacles = obstacles
        self.landmarks = landmarks
        self.danger_zones = danger_zones
        self.goals = goals
        self.state_positions = []
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                if (i, j) not in self.obstacles:
                    self.state_positions.append((i, j))

    @staticmethod
    def random_free_position(n, m, not_free_positions):
        """Returns a random (x,y) position in an n x m grid that is free."""
        while True:
            pos = (random.randint(1, n),
                   random.randint(1, m))
            if pos not in not_free_positions:
                return pos

    @staticmethod
    def generate_instance(n, m, k, l, d, g):
        """
        Returns a random instance of a Gridworld(n, m, k, g, l):
        n: grid horizontal dimension (x)
        m: grid vertical dimension (y)
        k: number of obstacles inside the grid (k < n x m)
        l: number of landmarks (l < n x m - k)
        d: number of danger_zones (d < n x m - k - l)
        g: number of goal positions (g < n x m - k)


        """

        if k >= n * m:
            raise ValueError("k is larger than n x m")

        if l >= n * m - k:
            raise ValueError("Not enough free spots for landmarks.")

        if d >= n * m - k - l:
            raise ValueError("Not enough free spots for landmarks.")

        if g >= n * m - k - l - d:
            raise ValueError("Not enough free spots for goals.")

        obstacles = []  # Store obstacles here
        for i in range(k):
            pos = GridMap.random_free_position(n, m, obstacles)
            obstacles.append(pos)

        not_landmarkable = copy.deepcopy(obstacles)

        landmarks = []
        for i in range(l):
            pos = GridMap.random_free_position(n, m, not_landmarkable)
            landmarks.append(pos)
            not_landmarkable.append(pos)

        not_danger_zone = copy.deepcopy(not_landmarkable)

        danger_zones = []
        for i in range(d):
            pos = GridMap.random_free_position(n, m, not_landmarkable)
            danger_zones.append(pos)
            not_danger_zone.append(pos)

        not_goalable = copy.deepcopy(not_danger_zone)

        goals = []
        for i in range(g):
            goal_pos = (n, int(m / 2))
            while goal_pos in not_goalable:
                goal_y_pos = random.sample(range(1, m + 1), k=1)[0]
                goal_pos = (n, goal_y_pos)
            goals.append(goal_pos)
            not_goalable.append(goal_pos)

        return GridMap(n, m, obstacles, landmarks, danger_zones, goals)

    def valid_pose(self, position):
        return position in self.state_positions

    def at_goal(self, position):
        return position in self.goals

    def at_landmark(self, position):
        return position in self.landmarks

    def at_danger_zone(self, position):
        return position in self.danger_zones

    def print_map(self):
        string = "\n"
        # Grid map
        for y in range(1, self.m + 1):
            for x in range(1, self.n + 1):
                char = "."
                if (x, y) in self.obstacles:
                    char = "#"
                if self.at_danger_zone((x, y)):
                    char = "D"
                if self.at_landmark((x, y)):
                    char = "L"
                if self.at_goal((x, y)):
                    char = "X"
                string += char
            string += "\n"
        print(string)

    # TODO: This could be improved for scaling down.
    def scale(self, x_scale=2, y_scale=2):
        """
        x_scale: scaling factor for the x-direction
        y_scale: scaling factor for the y-direction
        """

        def int_scale(i, scale_const):

            if scale_const > 1:
                if i < 1:
                    return i
                return math.ceil(i * scale_const)
            else:
                return math.floor(i * scale_const)

        _n = int_scale(self.n, x_scale)
        _m = int_scale(self.m, y_scale)
        _obstacles = []
        _landmarks = []
        _danger_zones = []
        _goals = []
        _state_positions = []

        for i in range(1, _n + 1):
            for j in range(1, _m + 1):
                _i, _j = int_scale(i, 1 / x_scale), int_scale(j, 1 / y_scale)
                if (_i, _j) in self.obstacles:
                    _obstacles.append((i, j))
                if (_i, _j) in self.landmarks:
                    _landmarks.append((i, j))
                if (_i, _j) in self.danger_zones:
                    _danger_zones.append((i, j))
                if (_i, _j) in self.goals:
                    _goals.append((i, j))
                if (_i, _j) in self.state_positions:
                    _state_positions.append((i, j))

        return GridMap(_n, _m, _obstacles, _landmarks, _danger_zones, _goals)
