""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node
from state_machine_node import STATE

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from std_msgs.msg import Bool, UInt8
from state_machine_node import STATE

from geometry_msgs.msg import Twist

class DriveCircleNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('drive_circle_node')
        self.timer = self.create_timer(0.1, self.periodic)
        self.state = STATE.IDLE

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(UInt8, '/current_state', self.update_state, 10)

        self.linearx_max = 0.3
        self.angularz_max = 0.3
        self.start_time = self.get_clock().now()

    def update_state(self, msg):
        try:
            self.state = STATE(msg.data)
        except ValueError as e:
            print(f'ERROR: Invalid State - {e}')


    def periodic(self):
        if self.state != STATE.CIRCLE:
            return
        
        current_time = self.get_clock().now()
        elapsed_seconds = (current_time - self.start_time).nanoseconds / 1e9

        twist_msg = Twist()

        # Linear velocity (fixed or increasing)
        twist_msg.linear.x = self.linearx_max  

        # Radius grows linearly with time
        r0 = 0.3
        k = 0.05
        radius = r0 + k * elapsed_seconds

        # Angular velocity from v/r
        twist_msg.angular.z = max(self.linearx_max / radius, 0.01)

        self.twist_pub.publish(twist_msg)


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """ 
    rclpy.init(args=args)      # Initialize communication with ROS
    node = DriveCircleNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()