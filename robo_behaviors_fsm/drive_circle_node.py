"""
This script implements a ROS 2 node that commands a robot to drive in an 
expanding spiral pattern.
"""
# Third-party imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

# Local application imports
from robo_behaviors_fsm.state_machine_node import STATE


class DriveCircleNode(Node):
    """
    Subscribes to a state topic and, when in the 'CIRCLE' state, publishes 
    Twist messages to drive a robot in an expanding spiral.
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and parameters."""
        super().__init__('drive_circle_node')

        # Parameters controlling the spiral's shape
        self.MAX_LINEAR_VELOCITY_X = 0.3  # m/s
        self.INITIAL_RADIUS = 0.3         # meters
        self.RADIUS_GROWTH_RATE = 0.05    # meters per second

        # Node's internal state, initialized to IDLE
        self.state = STATE.IDLE
        self.start_time = self.get_clock().now()

        # ROS 2 publishers, subscribers, and timers
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(UInt8, '/current_state', self.state_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Drive Circle Node has been initialized.')

    def state_callback(self, msg: UInt8):
        """
        Updates the node's internal state based on messages from the
        /current_state topic.

        Args:
            msg: The message containing the new state enum.
        """
        try:
            new_state = STATE(msg.data)
            if self.state != new_state:
                self.get_logger().info(f'State changed from {self.state.name} to {new_state.name}')
                self.state = new_state
                # Reset the timer when entering the CIRCLE state to start a new spiral
                if self.state == STATE.CIRCLE:
                    self.start_time = self.get_clock().now()
        except ValueError:
            self.get_logger().error(f'Received an invalid state value: {msg.data}')

    def timer_callback(self):
        """
        Periodically calculates and publishes Twist commands if in CIRCLE state.
        """
        if self.state != STATE.CIRCLE:
            return

        elapsed_seconds = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        twist_msg = Twist()
        twist_msg.linear.x = self.MAX_LINEAR_VELOCITY_X

        # Calculate radius and angular velocity for a spiral (v = w * r)
        current_radius = self.INITIAL_RADIUS + self.RADIUS_GROWTH_RATE * elapsed_seconds
        twist_msg.angular.z = -1.0 * (self.MAX_LINEAR_VELOCITY_X / max(current_radius, 0.01))

        # A debug log is useful for tuning without cluttering the main console
        self.get_logger().debug(
            f'Radius: {current_radius:.2f} m, '
            f'Vel: (lin={twist_msg.linear.x:.2f}, ang={twist_msg.angular.z:.2f})'
        )
        self.twist_publisher.publish(twist_msg)


def main(args=None):
    """
    Initializes the ROS 2 node, spins it to keep it running, and handles shutdown.
    
    Args:
        args (list, optional): List of command-line arguments for rclpy.
    """
    rclpy.init(args=args)
    node = DriveCircleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()