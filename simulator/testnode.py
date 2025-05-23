import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math
import tf_transformations

DEG2RAD = math.pi / 180.0
LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.5
FORWARD_DIST = 1
YAW_ROTATE = math.pi / 2


class TurtlebotCommandDrive(Node):
    def __init__(self):
        super().__init__('turtlebot_command_drive')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 50)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.ground_truth_callback, 50)

        self.current_yaw = 0.0
        self.true_yaw = 0.0
        self.start_yaw = None
        self.start_pos = None
        self.position = (0.0, 0.0)
        self.true_position = (0.0, 0.0)

        self.prev_odom_position = None
        self.total_odom_distance = 0.0
        self.linear_errors = []
        self.angular_errors = []

        self.state = 'idle'
        self.command_queue = []
        self.current_command = None
        self.command_step = 0

        self.pause_duration = 2.0
        self.pause_start_time = None
        self.resume_after_pause = None

        self.should_exit = False

        self.timer = self.create_timer(0.01, self.control_loop)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        ])
        self.current_yaw = yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.position = (x, y)

        # Incremental odometry distance
        if self.prev_odom_position is not None:
            dx = x - self.prev_odom_position[0]
            dy = y - self.prev_odom_position[1]
            delta = math.sqrt(dx ** 2 + dy ** 2)
            self.total_odom_distance += delta
        self.prev_odom_position = (x, y)

    def ground_truth_callback(self, msg):
        try:
            idx = msg.name.index("turtlebot3_burger")
            pos = msg.pose[idx].position
            ori = msg.pose[idx].orientation
            self.true_position = (pos.x, pos.y)
            _, _, yaw = tf_transformations.euler_from_quaternion([
                ori.x, ori.y, ori.z, ori.w
            ])
            self.true_yaw = yaw
        except ValueError:
            self.get_logger().warn("TurtleBot3 Burger not found in /gazebo/model_states")

    def control_loop(self):
        if self.state == 'pause':
            now = self.get_clock().now()
            if (now - self.pause_start_time).nanoseconds / 1e9 >= self.pause_duration:
                self.state = 'idle'
                if self.resume_after_pause:
                    self.resume_after_pause()
                    self.resume_after_pause = None
            return

        if self.current_command is None and self.command_queue:
            self.current_command = self.command_queue.pop(0)
            self.get_logger().info(f"Executing {self.current_command}")
            self.command_step = 0
            self.state = 'start'

        if self.current_command == 'Move-North':
            self.move_linear(FORWARD_DIST, 1.0)
        elif self.current_command == 'Move-South':
            self.move_linear(FORWARD_DIST, -1.0)
        elif self.current_command == 'Move-East':
            self.turn_move_return(-1)
        elif self.current_command == 'Move-West':
            self.turn_move_return(1)
        elif not self.command_queue and self.current_command is None:
            self.stop_robot()
            self.get_logger().info("All commands executed.")
            self.should_exit = True

    def move_linear(self, target_dist, direction):
        if self.state == 'start':
            self.start_pos = self.position
            self.state = 'moving'

        elif self.state == 'moving':
            dx = self.position[0] - self.start_pos[0]
            dy = self.position[1] - self.start_pos[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist >= target_dist:
                self.stop_robot()
                self.state = 'pause'
                self.pause_start_time = self.get_clock().now()
                self.resume_after_pause = self.finish_command

                # --- Linear error ---
                true_dx = self.true_position[0] - self.start_pos[0]
                true_dy = self.true_position[1] - self.start_pos[1]
                true_dist = math.sqrt(true_dx ** 2 + true_dy ** 2)
                linear_error = abs(true_dist - dist)
                self.linear_errors.append(linear_error)

                # --- Angular error ---
                yaw_error = abs(self._angle_diff(self.current_yaw, self.true_yaw))
                self.angular_errors.append(yaw_error)

                self.get_logger().info(
                    f"[LINEAR ERROR] Odom: {dist:.3f} m | Truth: {true_dist:.3f} m | Error: {linear_error:.4f} m"
                )
                self.get_logger().info(
                    f"[ANGULAR ERROR] Odom yaw: {math.degrees(self.current_yaw):.2f}° | "
                    f"Truth yaw: {math.degrees(self.true_yaw):.2f}° | "
                    f"Error: {math.degrees(yaw_error):.2f}°"
                )
            else:
                self.publish_cmd(LINEAR_VELOCITY * direction, 0.0)

    def finish_command(self):
        self.current_command = None

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
        """Returns the minimal signed angle difference between two angles in radians."""
        diff = a - b
        return math.atan2(math.sin(diff), math.cos(diff))


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCommandDrive()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    finally:
        if node.linear_errors:
            avg_linear = sum(node.linear_errors) / len(node.linear_errors)
            print(f"\nAverage linear error: {avg_linear:.4f} m over {len(node.linear_errors)} motions")

        if node.angular_errors:
            avg_angular_rad = sum(node.angular_errors) / len(node.angular_errors)
            avg_angular_deg = math.degrees(avg_angular_rad)
            print(f"Average angular error: {avg_angular_deg:.2f}° over {len(node.angular_errors)} motions")

        print(f"Total odometry distance traveled: {node.total_odom_distance:.4f} m")

        node.destroy_node()
        rclpy.shutdown()

