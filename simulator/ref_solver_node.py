import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ref_solver_clean import RobotAction, RobotAgent, RefSolver, create_robot_pomdp

class RefSolverNode(Node):
    def __init__(self):
        super().__init__('ref_solver_node')
        
        # Create publisher to send velocity commands to Gazebo
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber to receive sensor data (e.g., LaserScan) from Gazebo
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.update_agent,  # callback function to process the observation
            10)
        
        # Timer to call the plan function periodically (e.g., every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.execute_plan)
        
        # Initialize the POMDP agent and solver
        self.pomdp = create_robot_pomdp()
        self.solver = RefSolver()
        self.agent = self.pomdp.agent

        # Store the latest sensor reading (LaserScan)
        self.latest_observation = None

    def update_agent(self, msg):
        """Callback to update the agent based on sensor data from Gazebo."""
        # Convert LaserScan data to the format expected by the POMDP model
        observation = self.process_laser_scan(msg)
        
        # Update the agent's belief based on the observation
        if observation is not None:
            self.solver.update(self.agent, self.agent.cur_action, observation)

    def process_laser_scan(self, scan_data):
        """Process the LaserScan message to extract useful information."""
        # Example: Extract ranges and calculate minimum range
        min_range = min(scan_data.ranges)
        
        # Create a RobotObservation based on the sensor data
        return (0, min_range)  # Example observation (x=0, min_range)

    def execute_plan(self):
        """Call the POMDP solver's plan function to generate the next action for the agent."""
        if self.agent is not None:
            # Use the POMDP solver to generate the next action
            action = self.solver.plan(self.agent)

            # Convert the POMDP action into a Twist message for Gazebo
            twist = Twist()
            if action.name == "move_forward":
                twist.linear.x = 1.0  # Forward speed
                twist.angular.z = 0.0
            elif action.name == "turn_left":
                twist.linear.x = 0.0
                twist.angular.z = 1.0  # Angular speed to turn left
            elif action.name == "turn_right":
                twist.linear.x = 0.0
                twist.angular.z = -1.0  # Angular speed to turn right

            # Publish the action to control the robot in Gazebo
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RefSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

