import rclpy
import pomdp_py
import math
import random
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from datetime import datetime
from problems.gridworld.grid_map import GridMap
from problems.gridworld.domain import State
from problems.gridworld.utils import init_particles_belief
from problems.gridworld.models import (PolicyModel, TransitionModel, ObservationModel, RewardModel)
from problems.gridworld.problem import GridWorldProblem
from pomdp_py.representations.distribution.particles import Particles
from pomdp_py.algorithms.ref_solver_clean import RefSolver

# Edit here to change the robot movement in Gazebo 
DEG2RAD = math.pi / 180.0
LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.5
FORWARD_DIST = 1 - 0.03
YAW_ROTATE = math.pi / 2 - math.radians(1)
# ************************************************

class RefPOMDPNode(Node):
    def __init__(self):
        super().__init__('ref_pomdp_node')
        
        
	# Edit here to change the world, POMDP model and simulation parameters
        scale_param = 1
        n, m = 20, 20
        obstacles = [(2, i) for i in range(1, 4)] + [(2, i) for i in range(6, 13)] + [(2, i) for i in range(15, 19)] + \
                    [(8, i) for i in range(1, 4)] + [(10, i) for i in range(6, 13)] + [(10, i) for i in range(15, 19)] + \
                    [(14, i) for i in range(1, 7)] + [(14, i) for i in range(11, 21)] + \
                    [(j, 3) for j in range(2, 4)] + [(j, 3) for j in range(5, 11)] + [(j, 3) for j in range(12, 15)] + \
                    [(j, 8) for j in range(14, 21)] + [(j, 11) for j in range(14, 17)] + [(j, 11) for j in range(19, 21)] + \
                    [(j, 12) for j in range(2, 10)] + [(j, 15) for j in range(2, 10)] + [(j, 18) for j in range(2, 4)] + \
                    [(j, 18) for j in range(5, 10)]

        landmarks = [(15 + i, 12 + j) for i in range(3) for j in range(2)] + [(18 + i, 6 + j) for i in range(3) for j in range(2)] + \
                    [(12, 8), (3, 19), (1, 17), (1, 20), (10, 19), (13, 20), (10, 13), (11, 15), (14, 10), (18, 10)]

        danger_zones = [(20, 9), (20, 10)] + [(6 + i, 19) for i in range(3)] + [(4 + i, 4 + j) for i in range(6) for j in range(2)]
        goals = [(16, 12), (19, 7)]

        grid_map = GridMap(n, m, obstacles, landmarks, danger_zones, goals)
        simulations = 3000
        planning_time = 5
        discount_factor = 0.99

        init_states = [(1, 4)]
        init_belief = init_particles_belief(grid_map, init_states=init_states, num_particles=simulations)
        init_pos = (1, 4)
        init_state = State(init_pos, init_pos in grid_map.goals, init_pos in grid_map.landmarks, init_pos in grid_map.danger_zones)

        self.gridworld = GridWorldProblem(init_state, init_belief, grid_map, scale_param=scale_param)

        r_max, r_min = 200, -100
        R_max, R_min = 200, -100
        rew_scale = (r_min - r_max) / (R_min - R_max)
        rew_shift = r_max / rew_scale - R_max

        a_star = pomdp_py.AStar(self.gridworld)
        start = datetime.now()
        a_star_policy = a_star.a_star_policy(self.gridworld.agent)
        stop = datetime.now()
        self.gridworld.visualise_policy(a_star_policy)

        self.ref_solver = pomdp_py.RefSolver(max_depth=grid_map.n * 3,
                                             max_rollout_depth=grid_map.n * 3 * 1.2,
                                             planning_time=planning_time,
                                             fully_obs_policy=a_star_policy,
                                             rew_shift=rew_shift,
                                             rew_scale=rew_scale,
                                             exploration_const=0.9,
                                             discount_factor=0.99)
        # *************************************************************************************************************************                                             
                                             
        # ROS Publisher and Subscriber 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_yaw = 0.0
        self.start_yaw = None
        self.start_pos = None
        self.position = (0.0, 0.0)

        self.state = 'idle'
        self.planner_ready = True
        self.current_command = None
        self.command_step = 0

        self.pause_duration = 2.0
        self.pause_start_time = None
        self.resume_after_pause = None

        self.total_planning_time = 0.0
        self.planning_start_time = None
        self.nsteps_limit = 180
        self.nsteps_taken = 0
        self.gamma = 1.0
        self.cumulative_reward = 0.0
        self.cumulative_discounted_reward = 0.0

        self.should_exit = False
        self.timer = self.create_timer(0.01, self.control_loop)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def control_loop(self):
        if self.state == 'pause':
            now = self.get_clock().now()
            if (now - self.pause_start_time).nanoseconds / 1e9 >= self.pause_duration:
                self.state = 'idle'
                if self.resume_after_pause:
                    self.resume_after_pause()
                    self.resume_after_pause = None
            return

        if self.current_command is None and self.planner_ready:
            self.planner_ready = False
            self.planning_start_time = datetime.now()
            self.current_command = self.ref_solver.plan(self.gridworld.agent)
            planning_end_time = datetime.now()
            delta = (planning_end_time - self.planning_start_time).total_seconds()
            self.total_planning_time += delta
            self.get_logger().info(f"Planned and executing: {self.current_command} (took {delta:.2f}s)")
            self.command_step = 0
            self.state = 'start'

        if self.current_command == 'Move-NORTH':
            self.move_linear(FORWARD_DIST, 1.0)
        elif self.current_command == 'Move-SOUTH':
            self.move_linear(FORWARD_DIST, -1.0)
        elif self.current_command == 'Move-EAST':
            self.turn_move_return(-1)
        elif self.current_command == 'Move-WEST':
            self.turn_move_return(1)

    def move_linear(self, target_dist, direction):
        if self.state == 'start':
            self.start_pos = self.position
            self.state = 'moving'
        elif self.state == 'moving':
            dx = self.position[0] - self.start_pos[0]
            dy = self.position[1] - self.start_pos[1]
            self.dist = math.sqrt(dx ** 2 + dy ** 2)
            if self.dist >= target_dist:
                self.stop_robot()
                self.state = 'pause'
                self.pause_start_time = self.get_clock().now()
                self.resume_after_pause = self.finish_command
            else:
                self.publish_cmd(LINEAR_VELOCITY * direction, 0.0)

    def finish_command(self):
        reward = self.gridworld.env.state_transition(self.current_command, execute=True)
        # Use this observation to feed in the observation in Gazebo
        # observation = self.gridworld.env.provide_observation(self.postion, self.current_command)
        # Use this observation for Nav 1 problem
        observation = self.gridworld.env.provide_observation(self.gridworld.agent.observation_model, self.current_command)
        self.gridworld.agent.update_history(self.current_command, observation)
        self.ref_solver.update(self.gridworld.agent, self.current_command, observation)
        self.cumulative_reward += reward
        self.cumulative_discounted_reward += reward * self.gamma
        self.gamma *= self.ref_solver._discount_factor
        self.nsteps_taken += 1
        self.planner_ready = True
        self.current_command = None

        if self.nsteps_taken >= self.nsteps_limit:
            self.get_logger().info("Step limit reached.")
            self.should_exit = True
        elif self.gridworld.terminal(self.gridworld.env.state):
            self.get_logger().info("Goal reached!")
            self.should_exit = True

    def turn_move_return(self, direction):
        if self.command_step == 0:
            if self.state == 'start':
                self.start_yaw = self.current_yaw
                self.state = 'turning'
            elif self.state == 'turning':
                if abs(self._angle_diff(self.current_yaw, self.start_yaw)) >= YAW_ROTATE:
                    self.stop_robot()
                    self.command_step = 1
                    self.state = 'pause'
                    self.pause_start_time = self.get_clock().now()
                    self.resume_after_pause = lambda: self.set_state('start')
                else:
                    self.publish_cmd(0.0, ANGULAR_VELOCITY * direction)
        elif self.command_step == 1:
            self.move_linear(FORWARD_DIST, 1.0)
            if self.state == 'pause':
                self.command_step = 2
                self.state = 'start'
        elif self.command_step == 2:
            if self.state == 'start':
                self.start_yaw = self.current_yaw
                self.state = 'turning_back'
            elif self.state == 'turning_back':
                if abs(self._angle_diff(self.current_yaw, self.start_yaw)) >= YAW_ROTATE:
                    self.stop_robot()
                    self.command_step = 3
                    self.state = 'pause'
                    self.pause_start_time = self.get_clock().now()
                    self.resume_after_pause = self.finish_command
                else:
                    self.publish_cmd(0.0, ANGULAR_VELOCITY * -direction)

    def set_state(self, new_state):
        self.state = new_state

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def _angle_diff(self, a, b):
        diff = a - b
        return math.atan2(math.sin(diff), math.cos(diff))

def main(args=None):
    rclpy.init(args=args)
    node = RefPOMDPNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.should_exit:
            executor.spin_once(timeout_sec=0.1)
    finally:
        print("\n=== Trial Summary ===")
        print(f"Steps taken: {node.nsteps_taken}")
        print(f"Cumulative reward: {node.cumulative_reward}")
        print(f"Discounted cumulative reward: {node.cumulative_discounted_reward:.2f}")
        print(f"Goal reached? {node.gridworld.at_goal(node.gridworld.env.state)}")
        print(f"Total planning time: {node.total_planning_time:.2f}s")
        node.destroy_node()
        rclpy.shutdown()

