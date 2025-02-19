#!/usr/bin/env python3
"""
ROS2 Node: Debug POMDP Planner (Gazebo-based)

This node initializes a particle belief based on Gazebo, runs the POMDP solver,
and prints debugging information about the belief and computed actions.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import random
import math
import copy
import time

# -----------------------------------------------------------------------------
# POMDP-PY Imports
# -----------------------------------------------------------------------------
from pomdp_py.representations.distribution.particles import Particles
from pomdp_py.framework.basics import TransitionModel, ObservationModel
from pomdp_py.algorithms.ref_solver_clean import RefSolver

# -----------------------------------------------------------------------------
# Model Classes for the Robot
# -----------------------------------------------------------------------------
class State:
    """
    Represents the TurtleBot's state (x, y, θ).
    A state is terminal when x >= 5.0.
    """
    def __init__(self, x, y, theta, terminal=False):
        self.position = (x, y)
        self.theta = theta
        self.terminal = terminal

    def __str__(self):
        return f"State(x={self.position[0]:.2f}, y={self.position[1]:.2f}, theta={self.theta:.2f}, terminal={self.terminal})"

    def __eq__(self, other):
        return (isinstance(other, State) and 
                self.position == other.position and 
                math.isclose(self.theta, other.theta, abs_tol=1e-5) and 
                self.terminal == other.terminal)

    def __hash__(self):
        return hash((self.position, self.theta, self.terminal))

class Action:
    """Represents a discrete action with a name and a motion command."""
    def __init__(self, name, motion):
        self.name = name
        self.motion = motion

    def __str__(self):
        return self.name

class FullyObsGenerator:
    """
    Provides a simple action space for the agent.
    """
    def __init__(self):
        self.all_actions = [
            Action("forward", (0.5, 0.0)),
            Action("rotate_left", (0.0, 0.5)),
            Action("rotate_right", (0.0, -0.5))
        ]

class RewardModel:
    """
    Defines the reward function.
    """
    def reward_func(self, state, action):
        return 100.0 if state.terminal else (1.0 if action.name == "forward" else -0.5)

    def sample(self, state, action, next_state):
        return self.reward_func(state, action)

def sample_generative_model(agent, state, action):
    """
    Simulates the TurtleBot’s state transition.
    """
    x, y = state.position
    theta = state.theta

    if action.name == "forward":
        v = action.motion[0]
        new_x = x + v * math.cos(theta)
        new_y = y + v * math.sin(theta)
        new_theta = theta
    elif action.name == "rotate_left":
        new_x, new_y = x, y
        new_theta = theta + action.motion[1]
    elif action.name == "rotate_right":
        new_x, new_y = x, y
        new_theta = theta + action.motion[1]
    else:
        new_x, new_y, new_theta = x, y, theta

    terminal = new_x >= 5.0
    next_state = State(new_x, new_y, new_theta, terminal)
    reward = 100.0 if terminal else 0.0
    observation = "lidar_observation"
    nsteps = 1

    return next_state, observation, reward, nsteps

import pomdp_py.framework.basics as basics
basics.sample_generative_model = sample_generative_model

class CustomTransitionModel(TransitionModel):
    def sample(self, state, action):
        next_state, _, _, _ = sample_generative_model(None, state, action)
        return next_state

    def probability(self, next_state, state, action):
        return 1.0

class CustomObservationModel(ObservationModel):
    def sample(self, next_state, action):
        return "lidar_observation"

    def probability(self, observation, next_state, action):
        return 1.0

class RobotAgentWithParticles:
    """
    A robot agent that initializes its belief from Gazebo.
    """
    def __init__(self, node, num_particles=1000):
        self.node = node
        self.belief = self.init_particles_belief(num_particles)
        self.init_belief = self.belief
        self.tree = None
        self.history = tuple()
        self.all_actions = FullyObsGenerator().all_actions
        self.all_observations = ["lidar_observation"]
        self.reward_model = RewardModel()
        self.transition_model = CustomTransitionModel()
        self.observation_model = CustomObservationModel()

    def get_gazebo_robot_state(self):
        """Gets the robot's state from Gazebo."""
        client = self.node.create_client(GetModelState, "/gazebo/get_model_state")
        while not client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("Waiting for Gazebo service...")

        request = GetModelState.Request()
        request.model_name = "turtlebot3_burger"  # Change based on your robot model

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result():
            return future.result().pose.position.x, future.result().pose.position.y
        else:
            self.node.get_logger().error("Failed to get robot state from Gazebo!")
            return 0.0, 0.0

    def init_particles_belief(self, num_particles=1000):
        """Initializes a uniform belief distribution around the Gazebo robot position."""
        x, y = self.get_gazebo_robot_state()
        particles = []

        for _ in range(num_particles):
            noise_x = x + random.uniform(-0.5, 0.5)  # Spread particles near initial x
            noise_y = y + random.uniform(-0.5, 0.5)  # Spread particles near initial y
            particles.append(State(noise_x, noise_y, 0.0, terminal=False))

        return Particles(particles)

    def sample_belief(self):
        """Samples a state from the current belief distribution."""
        return self.belief.random()

# -----------------------------------------------------------------------------
# Debug Node: Runs the Solver Once and Prints All Input/Output
# -----------------------------------------------------------------------------
class DebugPOMDPPlannerNode(Node):
    def __init__(self):
        super().__init__('debug_ref_solver_node')
        self.agent = RobotAgentWithParticles(self)
        self.fully_obs_generator = FullyObsGenerator()
        self.solver = RefSolver(
            max_depth=5,
            max_rollout_depth=20,
            planning_time=1.0,
            discount_factor=0.95,
            exploration_const=0.2,
            rew_shift=0,
            rew_scale=0.01,
            fully_obs_generator=self.fully_obs_generator,
            fully_obs_policy=dict(),
            num_visits_init=0,
            z_val_init=1.0,
            r_est_init=0.0,
            G_init=0.0,
            show_progress=False
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("=== Debug: Running solver once ===")
        self.get_logger().info("Agent's initial belief (particles):")
        for idx, particle in enumerate(self.agent.belief.particles[:10]):  # Show only first 10
            self.get_logger().info(f"  Particle {idx}: {particle}")

        start_time = time.time()
        action = self.solver.plan(self.agent)
        elapsed = time.time() - start_time

        self.get_logger().info(f"Computed action: {action}")
        self.get_logger().info(f"Time taken by solver: {elapsed:.4f} seconds")

        self.timer.cancel()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DebugPOMDPPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

