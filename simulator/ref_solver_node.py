import rclpy
import pomdp_py
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from problems.gridworld.grid_map import GridMap
from problems.gridworld.domain   import State
from problems.gridworld.utils    import init_particles_belief
from problems.gridworld.models   import (
    PolicyModel,
    TransitionModel,
    ObservationModel,
    RewardModel,
)
from pomdp_py.representations.distribution.particles import Particles
from pomdp_py.algorithms.ref_solver_clean import RefSolver

class RefPOMDPNode(Node):
    def __init__(self):
        super().__init__('ref_pomdp_node')

        # 1) very simple 10×10 grid
        n, m = 10, 10
        grid_map = GridMap(n, m, obstacles=[],
                           landmarks=[(10,10)],
                           danger_zones=[],
                           goals=[(n//2, m//2)])

        # 2) initial state & belief
        init_pos   = (n//2, m//2)
        init_state = State(init_pos,
                           init_pos in grid_map.goals,
                           init_pos in grid_map.landmarks,
                           init_pos in grid_map.danger_zones)
        raw_belief = init_particles_belief(grid_map,
                                           init_states=[init_pos],
                                           num_particles=300)

        # 3) build POMDP
        agent = pomdp_py.Agent(raw_belief,
                               PolicyModel(grid_map),
                               TransitionModel(grid_map),
                               ObservationModel(grid_map),
                               RewardModel(grid_map))
        env   = pomdp_py.Environment(init_state,
                                     TransitionModel(grid_map),
                                     RewardModel(grid_map))
        self.pomdp = pomdp_py.POMDP(agent, env)

        # 4) precompute fully‐observed A★ policy
        a_star = pomdp_py.AStar(self.pomdp)
        self.fully_obs_policy = a_star.a_star_policy(self.pomdp.agent)

        # 5) make the RefSolver
        self.solver = RefSolver(
            max_depth=n*3,
            max_rollout_depth=int(n*3*1.2),
            planning_time=1.0,
            fully_obs_policy=self.fully_obs_policy,
            exploration_const=0.9,
            discount_factor=0.99
        )

        # 6) ROS pubs/subs
        self.cmd_pub = self.create_publisher(Twist,    '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.last_action = None
        self.create_timer(0.5, self.step)

    def odom_callback(self, msg):
        # convert continuous pose → discrete (flooring to cell), no extra class needed
        gx = math.floor(msg.pose.pose.position.x * 10)
        gy = math.floor(msg.pose.pose.position.y * 10)
        obs = (gx, gy)

        if self.last_action is not None:
            # update belief through the Agent API
            new_belief = self.pomdp.agent.update(self.last_action, obs)
            self.pomdp.agent.belief = new_belief
            self.pomdp.belief        = new_belief

    def step(self):
        # get a policy, pick the action for the MPE state
        policy = self.solver.plan(self.pomdp)
        mpe    = self.pomdp.belief.mpe()
        action = policy[mpe]
        self.last_action = action
        self.get_logger().info(f"Planning → {action}")

        # convert discrete action → Twist
        tw = Twist()
        if action.name == "Move-NORTH":
            tw.linear.x = 0.1
        elif action.name == "Move-SOUTH":
            tw.linear.x = -0.1
        elif action.name == "Move-EAST":
            tw.angular.z = -0.5; tw.linear.x = 0.1
        elif action.name == "Move-WEST":
            tw.angular.z = 0.5;  tw.linear.x = 0.1
        self.cmd_pub.publish(tw)

        if mpe.goal:
            self.get_logger().info("Goal reached – shutting down.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = RefPOMDPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

