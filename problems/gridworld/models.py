import random
import pomdp_py

from problems.gridworld.domain import MoveAction, MoveEast, MoveSouth, MoveWest, MoveNorth, State, Observation

EPSILON = 1e-30  # The effective mass for a zero proboability.
ETA = 0.1       # The likelihood of actuator failure.


class TransitionModel(pomdp_py.TransitionModel):

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.move_actions = [MoveNorth, MoveEast, MoveSouth, MoveWest]

    @classmethod
    def if_move_by(cls, grid_map, position, action):
        if isinstance(action, MoveAction):
            dx, dy = action.motion
            next_position = (position[0] + dx,
                             position[1] + dy)
            if grid_map.valid_pose(next_position):
                return next_position
        return position

    def turn(self, action, rotation=1):
        """
        rotation: the clockwise step change from the desired action
        e.g. 1 means rotate clockwise by 90 degrees, -1 means anti-clockwise by 90 degrees
        """

        # TODO: Remove or fix this.
        if not isinstance(action, MoveAction):
            print(action)
            raise ValueError("Action not a MoveAction.")

        if isinstance(action, MoveAction):
            return self.move_actions[
                (self.move_actions.index(action) + rotation) % len(self.move_actions)]

    # def next_position(self, position, motion):
    #     """Gets next the resulting position after taking a given motion."""
    #     aim_position = (position[0] + motion[0], position[1] + motion[1])
    #     if aim_position in self.state_positions:
    #         return aim_position
    #     else:
    #         return position

    def probability(self, next_state, state, action, normalized=False, **kwargs):

        if next_state.position not in self.grid_map.state_positions:
            print(next_state.position)
            raise ValueError("Invalid next state position.")
        if state.position not in self.grid_map.state_positions:
            print(state.position)
            raise ValueError("Invalid state position.")

        if state.terminal or state.goal or state.danger_zone:
            if next_state.__eq__(State(state.position, state.goal, state.landmark, state.danger_zone, True)):
                return 1 - EPSILON
            else:
                return EPSILON
        else:
            if next_state.position == TransitionModel.if_move_by(self.grid_map, state.position, action):
                return 1 - ETA
            elif next_state.position == TransitionModel.if_move_by(self.grid_map, state.position, self.turn(action, 1)):
                return ETA * 0.5
            elif next_state.position == TransitionModel.if_move_by(self.grid_map, state.position,
                                                                   self.turn(action, -1)):
                return ETA * 0.5
            else:
                return EPSILON

    def sample(self, state, action, argmax=False):

        if state.terminal or state.goal or state.danger_zone:
            return State(state.position, state.goal, state.landmark, state.danger_zone, True)

        g = self.grid_map

        # Possible next positions
        pos = TransitionModel.if_move_by(g, state.position, action)
        pos_r = TransitionModel.if_move_by(g, state.position, self.turn(action, 1))
        pos_l = TransitionModel.if_move_by(g, state.position, self.turn(action, -1))

        # Possible next states
        s = State(pos, g.at_goal(pos), g.at_landmark(pos), g.at_danger_zone(pos))
        s_r = State(pos_r, g.at_goal(pos_r), g.at_landmark(pos_r), g.at_danger_zone(pos_r))
        s_l = State(pos_l, g.at_goal(pos_l), g.at_landmark(pos_l), g.at_danger_zone(pos_l))

        # Probabilities
        p = self.probability(s, state, action)
        p_r = self.probability(s_r, state, action)
        p_l = self.probability(s_l, state, action)

        return random.choices([s, s_r, s_l], k=1, weights=[p, p_r, p_l])[0]

    def get_all_states(self):

        all_states = [State(pos,
                            self.grid_map.at_goal(pos),
                            self.grid_map.at_landmark(pos),
                            self.grid_map.at_danger_zone(pos))
                      for pos in self.grid_map.state_positions] + \
                     [State(pos,
                            self.grid_map.at_goal(pos),
                            self.grid_map.at_landmark(pos),
                            self.grid_map.at_danger_zone(pos),
                            True)
                      for pos in self.grid_map.state_positions
                      if self.grid_map.at_goal(pos) or self.grid_map.at_danger_zone(pos)]

        return all_states


# Observation Model
class ObservationModel(pomdp_py.ObservationModel):
    X_RANGE = [-1, 0, 1]
    Y_RANGE = [-1, 0, 1]
    NULL_OBSERVATION = Observation()

    def __init__(self, grid_map):
        self.grid_map = grid_map

    def probability(self, observation, next_state, action):
        # Independent of the action taken.
        if not next_state.landmark:
            if observation.__eq__(ObservationModel.NULL_OBSERVATION):
                return 1 - EPSILON
            else:
                return EPSILON

        else:
            x, y = next_state.position
            possible_readings = [(x + dx, y + dy) \
                                 for dx in ObservationModel.X_RANGE \
                                 for dy in ObservationModel.Y_RANGE]
            p = 1 / len(possible_readings)
            if observation.position_reading in possible_readings:
                return p
            else:
                return EPSILON

    def sample(self, next_state, action):

        if not isinstance(next_state, State):
            raise ValueError("Expected State object for <next_state>")

        if not next_state.landmark:
            return ObservationModel.NULL_OBSERVATION

        else:
            x, y = next_state.position
            possible_readings = [(x + dx, y + dy) \
                                 for dx in ObservationModel.X_RANGE \
                                 for dy in ObservationModel.Y_RANGE]
            return Observation(random.sample(possible_readings, 1)[0])

    def get_all_observations(self):

        observations = set()
        for next_state_pos in self.grid_map.state_positions:
            if not next_state_pos in self.grid_map.landmarks:
                observations.add(Observation())
            else:
                x, y = next_state_pos
                for i in ObservationModel.X_RANGE:
                    for j in ObservationModel.Y_RANGE:
                        observations.add(Observation((x + i, y + j)))

        return list(observations)


# Reward Model
class RewardModel(pomdp_py.RewardModel):

    def __init__(self, grid_map, scale_param=1):
        self.grid_map = grid_map
        self.scale_param = scale_param

    # No need to retain the redundancy on action and next_state.
    def reward_func(self, state, action):
        if state.terminal:
            return 0 * self.scale_param
        elif state.goal:
            return 200 * self.scale_param
        elif state.danger_zone:
            return -100 * self.scale_param
        else:
            dx, dy = action.motion
            next_position = (state.position[0] + dx,
                             state.position[1] + dy)
            if self.grid_map.valid_pose(next_position):
                return -1 * self.scale_param
            return -1 * self.scale_param

    def sample(self, state, action, next_state):
        # Deterministic
        return self.reward_func(state, action)


# Policy Model
class PolicyModel(pomdp_py.RolloutPolicy):

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.all_actions = {MoveNorth, MoveEast, MoveSouth, MoveWest}

    def sample(self, state, normalized=False, **kwargs):
        return random.sample(self.get_all_actions(state=state), 1)[0]

    def probability(self, action, state, normalized=False, **kwargs):
        raise NotImplementedError

    def argmax(self, state, normalized=False, **kwargs):
        """Returns the most likely reward"""
        raise NotImplementedError

    def get_all_actions(self, state=None, history=None):
        if state is None:
            return self.all_actions
        else:
            valid_actions = set()
            position = state.position
            for a in self.all_actions:
                aim_position = (position[0] + a.motion[0], position[1] + a.motion[1])
                if self.grid_map.valid_pose(aim_position):
                    valid_actions.add(a)

            return valid_actions

    def rollout(self, state, history=None):
        return random.sample(list(self.get_all_actions(state=state)), 1)[0]
