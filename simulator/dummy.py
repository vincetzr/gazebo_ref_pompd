import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class TurtleBotMoveTurn(Node):
    def __init__(self):
        super().__init__('turtlebot_move_turn')

        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive LaserScan messages
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer for periodic movement
        self.timer = self.create_timer(0.1, self.timer_callback)  # Update rate 10 Hz

        self.obstacle_detected = False
        self.last_valid_distance = float('inf')  # Store last valid distance
        self.obstacle_distance_threshold = 0.5  # Minimum distance to detect an obstacle (meters)
        self.smooth_move_speed = -0.1  # Reverse speed (m/sec), negative for reverse motion
        self.turning = False
        self.turn_start_time = None
        self.turn_duration = 2.0  # Time to complete a 90-degree turn

    def laser_callback(self, msg):
        if self.turning:
            return  # Skip processing during a turn

        # Get the front range data (center portion of the laser scan)
        front_ranges = msg.ranges[len(msg.ranges) // 4 : 3 * len(msg.ranges) // 4]  # Middle 180 degrees
        valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]

        if valid_ranges:
            self.last_valid_distance = min(valid_ranges)
            self.get_logger().info(f"Min distance in front: {self.last_valid_distance:.2f} m")

            # Trigger the turn if the obstacle is within the detection threshold (0.5 meters)
            if self.last_valid_distance < self.obstacle_distance_threshold:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.get_logger().warn("No valid LIDAR readings detected! Using last valid distance.")

    def timer_callback(self):
        twist = Twist()

        if self.turning:
            # Continue turning right
            twist.angular.z = -math.radians(45)  # Turn right at 45 degrees/sec
            self.publisher.publish(twist)

            # Check if the turn duration has elapsed
            if (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9 >= self.turn_duration:
                self.turning = False
                self.get_logger().info("Completed 90-degree turn.")
        elif self.obstacle_detected:
            # Stop and initiate turning
            twist = Twist()  # Stop any motion
            self.publisher.publish(twist)

            self.turning = True
            self.turn_start_time = self.get_clock().now()
            self.get_logger().info("Obstacle detected! Stopping and starting right 90-degree turn.")
        else:
            # Move backward if no obstacle is detected
            twist.linear.x = self.smooth_move_speed  # Negative value for reverse movement
            self.publisher.publish(twist)

        # Always publish the twist message
        if not self.turning:
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    node = TurtleBotMoveTurn()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

