import numpy as np
import pomdp_py

from problems.gridworld.domain import State
from problems.gridworld.models import PolicyModel, TransitionModel, ObservationModel, RewardModel


class GridWorldProblem(pomdp_py.POMDP):

    def __init__(self, init_state, init_belief, grid_map, scale_param=1):
        self.init_state = init_state  # the true starting state of the environment
        self.init_belief = init_belief  # agent's initial belief
        self.grid_map = grid_map
        self.scale_param = scale_param

        agent = pomdp_py.Agent(self.init_belief,
                               PolicyModel(self.grid_map),
                               TransitionModel(self.grid_map),
                               ObservationModel(self.grid_map),
                               RewardModel(self.grid_map, scale_param))
        env = pomdp_py.Environment(self.init_state,
                                   TransitionModel(self.grid_map),
                                   RewardModel(self.grid_map, self.scale_param))
        super().__init__(agent, env, name="GridWorldProblem")

    def at_goal(self, state):
        return state.goal

    def terminal(self, state):
        return state.terminal

    def obs_support(self, observation):

        x, y = observation.position_reading
        return [State((x + i, y + j),
                      (x + i, y + j) in self.grid_map.goals,
                      (x + i, y + j) in self.grid_map.landmarks,
                      (x + i, y + j) in self.grid_map.danger_zones) \
                for i in ObservationModel.X_RANGE \
                for j in ObservationModel.Y_RANGE]

    def print_state(self):
        string = "\n"
        robot_position = self.env.state.position
        # Grid map
        for y in range(1, self.grid_map.m + 1):
            for x in range(1, self.grid_map.n + 1):
                char = "."
                if self.grid_map.at_danger_zone((x, y)):
                    char = "D"
                if self.grid_map.at_landmark((x, y)):
                    char = "L"
                if self.grid_map.at_goal((x, y)):
                    char = "X"
                if (x, y) in self.grid_map.obstacles:
                    char = "#"
                if (x, y) == robot_position:
                    char = "R"
                string += char
            string += "\n"
        print(string)

    def visualise_policy(self, policy, values=None):
        string = "\n"
        # Grid map

        print("Policy:")

        for y in range(1, self.grid_map.m + 1):
            for x in range(1, self.grid_map.n + 1):
                char = "."
                if (x, y) in self.grid_map.obstacles:
                    char = "#"
                elif self.grid_map.at_goal((x, y)):
                    char = "X"
                else:
                    action = policy.get(State((x, y),
                                              self.grid_map.at_goal((x, y)),
                                              self.grid_map.at_landmark((x, y)),
                                              self.grid_map.at_danger_zone((x, y))
                                              )
                                        )
                    if action is None:
                        char = "?"
                    else:
                        if action.name == 'Move-NORTH':
                            char = "^"
                        if action.name == 'Move-SOUTH':
                            char = "v"
                        if action.name == 'Move-EAST':
                            char = ">"
                        if action.name == 'Move-WEST':
                            char = "<"
                string += char
            string += "\n"

        print(string)

        # print values if provided
        ###########################################################
        if values is not None:

            print("Values:\n")

            np.set_printoptions(precision=3)
            A = np.full((self.grid_map.m, self.grid_map.n), float("inf"))
            for state in values.keys():
                pos = state.position
                if values[state] is not None:
                    A[pos[1] - 1][pos[0] - 1] = values[state]
                else:
                    A[pos[1] - 1][pos[0] - 1] = float('inf')

            print(A)
        ###########################################################
