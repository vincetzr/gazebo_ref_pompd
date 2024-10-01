"""Implementation of the standard value iteration algorithm for MDPs."""

class ValueIterationMDP():
    
    def __init__(self, discount_factor=0.95, epsilon=1e-9):
        
        self.discount_factor = discount_factor
        self.epsilon = epsilon

    def generate_policy(self, agent):
        actions = agent.all_actions
        states = agent.all_states
        reward = agent.reward_model.sample

        # Initialise values
        values = {s : 0 for s in states}

        while True:      
            values_pre_iter = values.copy()
            diff = 0 # reset to zero
            
            for s in states:
                values[s] = max([sum([agent.transition_model.probability(ns, s, a) * 
                                     (reward(s, a, ns) + self.discount_factor * values_pre_iter[ns]) for ns in states]) for a in actions])

                diff = max(diff, abs(values_pre_iter[s] - values[s]))
                
            if diff < self.epsilon:
                break

        # Return the optimal action:
        policy = {}
        for s in states:

            if agent.policy_model.get_all_actions(state=s) == set():
                # i.e. the state has no valid actions
                raise ValueError("The problem instance has a state with no valid actions.  Cannot generate a policy for every state.")

            options = {a : sum([agent.transition_model.probability(ns, s, a) * 
                                 (reward(s, a, ns) + self.discount_factor *\
                                  values[ns]) for ns in states]) for a in \
                                 agent.policy_model.get_all_actions(state=s)}

            policy[s] = max(options, key=options.get)

        return policy, values