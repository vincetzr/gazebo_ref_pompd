import copy
import math
import random
import pomdp_py
from datetime import datetime, timedelta

from problems.gridworld.domain import State


def init_particles_belief(grid_map, init_states=None, num_particles=1000):
    """Samples an initial belief particle distribution where
    it is assumed that initial states can't be goal states.
    Assumes the distribution is uniform over init_states provided.

    grid_map:       a GridMap object.
    init_states:    a list of initial state positions. If None, defaults to the
                    state_positions in grid_map provided.
    num_particles:  the number of particles used to represent the distribution.
    """
    particles = []

    if init_states:
        possible_positions = init_states
    else:
        possible_positions = [s for s in grid_map.state_positions if s not in grid_map.goals]

    for p in range(num_particles):
        pos = random.sample(possible_positions, 1)[0]
        particles.append(State(pos, pos in grid_map.goals, pos in grid_map.landmarks))

    init_belief = pomdp_py.Particles(particles)
    return init_belief


def test_planner(gridworld, planner, nsteps=3, discount=0.95):
    """
    Runs the action-feedback loop of GridWorld problem POMDP
    """

    # TODO: Define these functions elsewhere.
    def expand_support(d, support, epsilon=1e-9):
        c = {}
        for omega in support:
            if d.get(omega) is None:
                c[omega] = epsilon
            else:
                c[omega] = d[omega]

        return c

    def kl(d1, d2):

        if not isinstance(d1, dict) or not isinstance(d2, dict):
            raise ValueError("One of the inputs is not a dict.")

        if len(d1) != len(d2):
            raise ValueError("Inputs have different length.")

        kl = 0.0
        for omega in d1.keys():
            if d1[omega] < 0.0 or d2[omega] < 0.0:
                raise ValueError('An input distribution has negative probability.')
            if d2[omega] == 0.0 and d1[omega] > 0.0:
                raise ValueError("d1 has positive support where d2 does not.")
            if d2[omega] > 0.0 and d1[omega] > 0.0:
                kl += d1[omega] * math.log(d1[omega] / d2[omega])

        return kl

    gamma = 1.0
    cumulative_reward = 0.0
    cumulative_discounted_reward = 0.0

    print("\n====== Initial Belief ======\n", gridworld.agent.cur_belief)
    print("True State:", gridworld.env.state)
    gridworld.print_state()

    for i in range(nsteps):
        # Step 2: Agent plans action
        action = planner.plan(gridworld.agent)

        # Step 3: Environment transitions state
        reward = gridworld.env.state_transition(action, execute=True)

        # Step 4: Agent receives observation
        observation = gridworld.env.provide_observation(gridworld.agent.observation_model, action)

        # Step 5: Agent updates history and belief.
        gridworld.agent.update_history(action, observation)
        planner.update(gridworld.agent, action, observation)
        cumulative_reward += reward
        cumulative_discounted_reward += reward * gamma
        gamma *= discount

        # Print summary data of each step
        print("\n====== Step %d ======" % (i + 1))

        if isinstance(planner, pomdp_py.RefSolver):
            print("Estimated Optimal Stochastic Policy:", planner._u_opt)
            print("Fully Observed Recommendation Given Belief:", planner._est_fully_obs_policy.get_histogram())
            print("KL(Est Opt Policy || Est FO Policy):",
                  kl(planner._u_opt, expand_support(planner._est_fully_obs_policy.get_histogram(),
                                                    gridworld.agent.all_actions)))
            # print("Embedded Recommendation Given Belief:", planner._est_fully_obs_policy)
            # print("KL(Est Opt Policy || Est FO Policy):",
            #       kl(planner._u_opt, expand_support(planner._est_fully_obs_policy,
            # gridworld.agent.all_actions)))
        print("Action Taken:", action)
        print("Observation:", observation)
        print("Resulting Belief:", gridworld.agent.cur_belief)
        print("True State:", gridworld.env.state)
        gridworld.print_state()
        print("Step Reward:", reward)
        print("Reward (Cumulative):", cumulative_reward)
        print("Reward (Cumulative Discounted):", cumulative_discounted_reward)
        if isinstance(planner, pomdp_py.POUCT) \
                or isinstance(planner, pomdp_py.RefSolver):
            print("__num_sims__: %d" % planner.last_num_sims)
            print("__plan_time__: %.5f" % planner.last_planning_time)
        if isinstance(planner, pomdp_py.PORollout):
            print("__best_reward__: %d" % planner.last_best_reward)
        if isinstance(gridworld.agent.cur_belief, pomdp_py.Histogram):
            new_belief = pomdp_py.update_histogram_belief(gridworld.agent.cur_belief,
                                                          action, observation,
                                                          gridworld.agent.observation_model,
                                                          gridworld.agent.transition_model)
            gridworld.agent.set_belief(new_belief)

        if gridworld.terminal(gridworld.env.state):
            print("\n====== TRIAL ENDED! ======")
            next_state, reward = gridworld.env.state_transition(action, execute=False)  # which action does not matter
            print("Step Reward:", reward)
            print("Total Reward (Cumulative):", cumulative_reward + reward)
            print("Total Reward (Cumulative Discounted):", cumulative_discounted_reward + reward * gamma)
            break

    return cumulative_reward, cumulative_discounted_reward


def benchmark_planner(gridworld,
                      planner,
                      trials=10,
                      nsteps=100,
                      discount_factor=0.99):
    """
    Computes key statistics to measure the performance of a planner in a given
    grid world. A successful trial is one that reaches a goal state.
    Statistics on the reward are also returned.

    gridworld:           A Gridworld object.
    planner:            A pomdp_py.Planner object.
    trials:             Number of times planner is run per simulation count.
    nsteps:             The number of steps executed by planner.
    discount_factor:    The discount_factor belonging to [0, 1).

    """

    errors = dict()
    sans_errors = 0
    tdr = 0
    min_crd = math.inf
    max_crd = -math.inf
    successful_trials = 0
    total_run_time = timedelta()
    total_successful_run_time = timedelta()
    success_list = []
    reward_discounted_list = []
    trial_list = []

    for trial in range(1, trials + 1):

        print(f"\n====== TRIAL STARTING-{trial} ======")
        _problem = copy.deepcopy(gridworld)
        try:
            # Test and time
            start_time = datetime.now()
            cr, crd = test_planner(_problem, planner, nsteps=nsteps, discount=discount_factor)
            stop_time = datetime.now()

            total_run_time = total_run_time + stop_time - start_time

            tdr += crd
            reward_discounted_list.append(crd)
            trial_list.append(trial)
            if crd < min_crd:
                min_crd = crd
            if crd > max_crd:
                max_crd = crd
            sans_errors += 1

            if _problem.at_goal(_problem.env.state):
                successful_trials += 1
                total_successful_run_time = total_successful_run_time + stop_time - start_time
                success_list.append(1)
            else:
                success_list.append(0)

            errors[trial] = "N/A"

        except Exception as e:
            errors[trial] = e

    results = {
        "trials without errors": sans_errors,
        "successful trials": successful_trials,
        "avg cum disc reward": tdr / sans_errors \
            if sans_errors > 0 else "All trials suffered errors!",
        "max cum disc reward": max_crd,
        "min cum disc reward": min_crd,
        "avg run time": total_run_time / trials,
        "avg successful run time": total_successful_run_time / successful_trials \
            if successful_trials > 0 else "No successful trials...",
        "errors": errors,
        "reward discounted list": reward_discounted_list,
        "success list": success_list,
        "trial list": trial_list,
        "total trials": trials,
    }

    return results
