""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

class WallFollowingNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('wall_following_node')
        self.timer = self.create_timer(0.1, self.publish_twist_command)
        self.stop_wall_following = False
        self.found_wall = False
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/turn_off_wall_following', self.turn_off_circle_callback, 10)
        self.found_wall_pub = self.create_publisher(Bool, 'found_wall', 10)

        self.linearx_max = 0.3
        self.start_time = self.get_clock().now()

    def turn_off_circle_callback(self, msg:Bool):
        self.stop_wall_following = msg.data
        if self.stop_wall_following:
            self.get_logger().warn("Turning off wall following mode")

    def find_wall(self):
        # LIDAR CODER HEREEE
        a=1
        if a==2:
            self.found_wall = True
        self.found_wall_pub.publish(self.found_wall)

    def publish_twist_command(self):
        if self.stop_wall_following or not self.found_wall:
            return
        # write code here??
        # use lidar to turn to wall and then go in a straight line/use PID to correct


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = WallFollowingNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()