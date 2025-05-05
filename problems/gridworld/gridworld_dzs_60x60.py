import argparse
import random
from datetime import datetime
import pomdp_py

from grid_map import GridMap
from domain import State
from problem import GridWorldProblem
from utils import init_particles_belief, benchmark_planner


def main(trials_count):
    scale_param = 3
    # ***** SMALL GRID MAP *****
    n, m = 20, 20
    obstacles = [] \

    landmarks = []

    danger_zones = []

    goals = [(16, 12)]
    grid_map = GridMap(n, m, obstacles, landmarks, danger_zones, goals)

    grid_map = grid_map.scale(scale_param, scale_param)
    grid_map.danger_zones = [pos for pos in grid_map.danger_zones if pos not in \
                             [(12 + i, 14 + j) for i in range(18) for j in range(2)]]
    grid_map.obstacles = grid_map.obstacles \
                         + [(6 + i, 1 + j) for i in range(3) for j in range(2)] \
                         + [(24 + i, 1 + j) for i in range(3) for j in range(2)] \
                         + [(42 + i, 1 + j) for i in range(3) for j in range(2)]

    # ***** BENCHMARK PARAMETERS *****
    simulations = 3000
    planning_time = 30
    trials = trials_count
    nsteps = 180
    discount_factor = 0.99
    # ********************************

    init_states = [(1 + i, 13 + j) for i in range(2) for j in range(3)] \
                  + [(1 + i, 40 + j) for i in range(2) for j in range(3)]
    init_belief = init_particles_belief(grid_map, init_states=init_states, num_particles=simulations)

    # init_pos = (1,4)
    init_pos = random.sample(init_states, 1)[0]
    init_state = State(init_pos,
                       init_pos in grid_map.goals,
                       init_pos in grid_map.landmarks,
                       init_pos in grid_map.danger_zones)

    gridworld = GridWorldProblem(init_state, init_belief, grid_map, scale_param=scale_param)

    r_max = 60
    r_min = -30
    R_max = 600
    R_min = -300

    rew_scale = (r_min - r_max) / (R_min - R_max)
    rew_shift = r_max / rew_scale - R_max

    print("\n\n***** PROBLEM DEFINITION *****\n")
    print("Gridworld size (n x m):\t\t", grid_map.n, "x", grid_map.m)
    print("True initial state:\t\t\t", init_state.position)
    print("Agent's initial belief:", init_belief.get_histogram())
    print("Goal state(s):\t\t\t\t", grid_map.goals)
    print("Rmax (goal state):\t\t\t\t", R_max)
    print("Rmin (danger zone): \t\t\t\t\t", R_min)

    gridworld.print_state()

    print("\n\n***** GENERATING FULLY OBSERVED POLICY GENERATOR *****")

    # ******* A Star *******
    a_star = pomdp_py.AStar(gridworld)

    print("\n\n***** GENERATING FULLY OBSERVED POLICY *****\n")

    start = datetime.now()

    a_star_policy = a_star.a_star_policy(gridworld.agent)

    # class RolloutPolicy(PolicyModel):
    #     def rollout(self, state, history):
    #         """rollout(self, State state, tuple history=None)"""
    #         pass

    class AStarRollout(pomdp_py.RolloutPolicy):
        """A rollout policy that chooses actions using an a* policy."""

        def __init__(self, a_star_policy):
            self._a_star_policy = a_star_policy

        def rollout(self, state, history=None):
            return self._a_star_policy[state]

    a_star_getter = AStarRollout(a_star_policy)

    stop = datetime.now()

    gridworld.visualise_policy(a_star_policy)

    print("Preprocessing time fully observed policy:", stop - start)

    print("\n\n***** BUILDING PLANNER(S) *****\n")

    ref_solver = pomdp_py.RefSolver(max_depth=90,
                                    max_rollout_depth=180,
                                    planning_time=planning_time,
                                    # num_sims=simulations,
                                    fully_obs_policy=a_star_policy,
                                    # fully_obs_generator=a_star,
                                    rew_shift=rew_shift,
                                    rew_scale=rew_scale,
                                    # rew_shift=0,
                                    # rew_scale=1/3,
                                    exploration_const=0.5,
                                    discount_factor=discount_factor)

    # uniform rollout
    # pomcp = pomdp_py.POMCP(max_depth=grid_map.n * 3,
    #                         planning_time=planning_time,
    #                         # num_sims=simulations,
    #                         discount_factor=0.99,
    #                         exploration_const=1000,
    #                         rollout_policy=gridworld.agent.policy_model)

    # A* rollout
    pomcp_a_star = pomdp_py.POMCP(max_depth=180,
                                  planning_time=planning_time,
                                  # num_sims=simulations,
                                  discount_factor=discount_factor,
                                  exploration_const=300,
                                  rollout_policy=a_star_getter)

    # print("\n\n***** RUNNING ONLINE PLANNER(S) *****\n")

    # cr, crd = test_planner(gridworld, ref_solver, nsteps=nsteps, discount=discount_factor)

    # cr, crd = test_planner(gridworld, pomcp, nsteps=nsteps, discount=discount_factor)

    # cr, crd = test_planner(gridworld, pomcp_a_star, nsteps=nsteps, discount=discount_factor)

    print("\n\n***** RUNNING BENCHMARKS *****\n")

    print("\nPARAMETERS:\n")
    print("Simulations:", simulations)
    print("Planning time:", planning_time)
    print("Trials:", trials)
    print("Planning horizon:", nsteps)
    print("Discount factor:", discount_factor)

    print("\nRefSolver:\n----------------------------")
    print("Reward shift:", ref_solver._rew_shift)
    print("Reward scale:", ref_solver._rew_scale)
    print("Exploration constant:", ref_solver._exploration_const)
    print("Max tree depth:", ref_solver._max_depth)
    print("Max rollout depth:", ref_solver._max_rollout_depth)
    print("----------------------------")

    # print("\nPOMCP (Uniform rollout):\n----------------------------")
    # print("Exploration constant:", pomcp._exploration_const)
    # print("Max depth:", pomcp._max_depth)
    # print("----------------------------")

    print("\nPOMCP (A* rollout):\n----------------------------")
    print("Exploration constant:", pomcp_a_star._exploration_const)
    print("Max depth:", pomcp_a_star._max_depth)
    print("----------------------------")

    # ********************************

    results_1 = benchmark_planner(gridworld, ref_solver,
                                  trials=trials,
                                  nsteps=nsteps,
                                  discount_factor=discount_factor)

    results_3 = benchmark_planner(gridworld, pomcp_a_star,
                                  trials=trials,
                                  nsteps=nsteps,
                                  discount_factor=discount_factor)

    print("\n\n***** RESULTS *****\n")

    print("\nResults RefSolver:")
    for i, v in results_1.items():
        print(i + ":", v)

    print("\nResults POMCP (A* rollout):")
    for i, v in results_3.items():
        print(i + ":", v)

    print("\n\nPreprocessing time fully observed policy:", stop - start, "\n")

    gridworld.visualise_policy(ref_solver._fully_obs_policy)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=str, default="100", help="Random seed")
    parser.add_argument("--trials", type=int, default=1, help="Number of trials")

    args = parser.parse_args()
    print(f"Arguments: {args}")

    random.seed(args.seed)

    main(args.trials)
