import rclpy
import pomdp_py
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from datetime import datetime

from problems.gridworld.grid_map import GridMap
from problems.gridworld.domain   import State
from problems.gridworld.utils    import init_particles_belief
from problems.gridworld.models   import (
    PolicyModel,
    TransitionModel,
    ObservationModel,
    RewardModel,
)
from problems.gridworld.problem import GridWorldProblem
from pomdp_py.representations.distribution.particles import Particles
from pomdp_py.algorithms.ref_solver_clean import RefSolver

class RefPOMDPNode(Node):
    def __init__(self):
        super().__init__('ref_pomdp_node')

        # scaling for benchmarking
        scale_param = 3

        # ***** SMALL GRID MAP *****
        n, m = 20, 20
        obstacles = [
            (2, i) for i in range(1, 4)
        ] + [
            (2, i) for i in range(6, 13)
        ] + [
            (2, i) for i in range(15, 19)
        ] + [
            (8, i) for i in range(1, 4)
        ] + [
            (10, i) for i in range(6, 13)
        ] + [
            (10, i) for i in range(15, 19)
        ] + [
            (14, i) for i in range(1, 7)
        ] + [
            (14, i) for i in range(11, 21)
        ] + [
            (j, 3) for j in range(2, 4)
        ] + [
            (j, 3) for j in range(5, 11)
        ] + [
            (j, 3) for j in range(12, 15)
        ] + [
            (j, 8) for j in range(14, 21)
        ] + [
            (j, 11) for j in range(14, 17)
        ] + [
            (j, 11) for j in range(19, 21)
        ] + [
            (j, 12) for j in range(2, 10)
        ] + [
            (j, 15) for j in range(2, 10)
        ] + [
            (j, 18) for j in range(2, 4)
        ] + [
            (j, 18) for j in range(5, 10)
        ]

        landmarks = [
            (15 + i, 12 + j) for i in range(3) for j in range(2)
        ] + [
            (18 + i, 6 + j) for i in range(3) for j in range(2)
        ] + [
            (12, 8), (3, 19), (1, 17), (1, 20),
            (10, 19), (13, 20), (10, 13), (11, 15)
        ] + [
            (14, 10), (18, 10)
        ]

        danger_zones = [
            (20, 9), (20, 10)
        ] + [
            (6 + i, 19) for i in range(3)
        ] + [
            (4 + i, 4 + j) for i in range(6) for j in range(2)
        ]

        goals = [(16, 12), (19, 7)]

        # build and scale map
        grid_map = GridMap(n, m,
                           obstacles,
                           landmarks,
                           danger_zones,
                           goals)
        grid_map = grid_map.scale(scale_param, scale_param)
        grid_map.danger_zones = [
            pos for pos in grid_map.danger_zones
            if pos not in [(12 + i, 14 + j) for i in range(18) for j in range(2)]
        ]
        grid_map.obstacles = grid_map.obstacles + [
            (6 + i, 1 + j) for i in range(3) for j in range(2)
        ] + [
            (24 + i, 1 + j) for i in range(3) for j in range(2)
        ] + [
            (42 + i, 1 + j) for i in range(3) for j in range(2)
        ]

        # ***** BENCHMARK PARAMETERS *****
        simulations = 3000
        planning_time = 30
        trials = 1  
        nsteps = 180
        discount_factor = 0.99

        # initial belief
        init_states = [
            (1 + i, 13 + j) for i in range(2) for j in range(3)
        ] + [
            (1 + i, 40 + j) for i in range(2) for j in range(3)
        ]
        init_belief = init_particles_belief(
            grid_map,
            init_states=init_states,
            num_particles=simulations
        )

        init_pos = random.choice(init_states)
        init_state = State(
            init_pos,
            init_pos in grid_map.goals,
            init_pos in grid_map.landmarks,
            init_pos in grid_map.danger_zones
        )

        # wrap into POMDP
        self.gridworld = GridWorldProblem(
            init_state,
            init_belief,
            grid_map,
            scale_param=scale_param
        )

        r_max, r_min = 60, -30
        R_max, R_min = 600, -300
        rew_scale = (r_min - r_max) / (R_min - R_max)
        rew_shift = r_max / rew_scale - R_max

        print("\n\n***** PROBLEM DEFINITION *****\n")
        self.gridworld.print_state()

        # fully observed A* policy
        a_star = pomdp_py.AStar(self.gridworld)
        start = datetime.now()
        a_star_policy = a_star.a_star_policy(self.gridworld.agent)
        stop = datetime.now()
        self.gridworld.visualise_policy(a_star_policy)
        print("Preprocessing time fully observed policy:", stop - start)

        # RefSolver instantiation
        self.ref_solver = pomdp_py.RefSolver(
            max_depth=90,
            max_rollout_depth=180,
            planning_time=planning_time,
            fully_obs_policy=a_star_policy,
            rew_shift=rew_shift,
            rew_scale=rew_scale,
            exploration_const=0.5,
            discount_factor=discount_factor
        )

        # ROS pubs & subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.last_action = None
        self.create_timer(0.5, self.step)

    def odom_callback(self, msg):
        # 1) read continuous world coords
        x_world = msg.pose.pose.position.x
        y_world = msg.pose.pose.position.y

        # 2) shift so [-2.5,2.5] → [0,5.0], then scale so [0,5.0]→[0,20)
        gx = math.floor((x_world + 2.5) / 0.25) + 1
        gy = math.floor((y_world + 2.5) / 0.25) + 1

        # 3) clamp into [1..20]
        gx = max(1, min(20, gx))
        gy = max(1, min(29, gy))
        
        obs = (gx, gy)
        
        # print / log every time we get an observation
        self.get_logger().info(f"Got observation: {obs}")

        if self.last_action is not None:
            self.ref_solver.update(self.gridworld.agent, self.last_action, obs)


    def step(self):
        action = self.ref_solver.plan(self.gridworld.agent)
        self.last_action = action
        self.get_logger().info(f"Planning → {action}")

        tw = Twist()
        if action.name == "Move-NORTH":
            tw.linear.x = 0.1
        elif action.name == "Move-SOUTH":
            tw.linear.x = -0.1
        elif action.name == "Move-EAST":
            tw.angular.z = -0.5; tw.linear.x = 0.1
        elif action.name == "Move-WEST":
            tw.angular.z = 0.5; tw.linear.x = 0.1
        self.cmd_pub.publish(tw)


def main():
    rclpy.init()
    node = RefPOMDPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


