"""
This script implements a ROS 2 node for a simple person-following behavior.
It uses LaserScan data to detect the closest object, calculates the error
to a target distance, and publishes velocity commands to follow it.
"""
# Standard library imports
import math

# Third-party imports
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8

# Local application imports
from robo_behaviors_fsm.state_machine_node import STATE


class PersonFollowingNode(Node):
    """
    Follows the closest detected object using LaserScan data.

    This node implements a simple proportional controller. It subscribes to a
    laser scan topic to find the closest point, calculates the distance and
    angle error to a predefined setpoint, and publishes Twist messages to
    minimize these errors when in the PERSON_FOLLOW state.
    """
    def __init__(self):
        """Initializes the node, parameters, and ROS communications."""
        super().__init__('person_following_node')

        # --- Control Parameters and Constants ---
        self.TARGET_DISTANCE = 0.7              # meters
        self.LINEAR_P_GAIN = 0.1                # Proportional gain for linear velocity
        self.ANGULAR_P_GAIN = 0.1               # Proportional gain for angular velocity
        self.PERSON_DETECTION_RANGE = 2.0       # Max distance to consider a person "found"

        # --- Internal State Variables ---
        self.state = STATE.IDLE
        self.distance_error = 0.0
        self.angle_error = 0.0
        self.min_forward_distance = float('inf')

        # --- ROS 2 Communications ---
        self.create_subscription(UInt8, '/current_state', self.state_callback, 10)
        self.create_subscription(LaserScan, '/stable_scan', self.scan_callback, 10)
        self.found_person_publisher = self.create_publisher(Bool, '/found_person', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.25, self.timer_callback) # 4 Hz control loop

        self.get_logger().info('Person Following Node has been initialized.')

    def state_callback(self, msg: UInt8):
        """
        Updates the node's internal state based on the /current_state topic.

        Args:
            msg: The message containing the new state enum.
        """
        try:
            new_state = STATE(msg.data)
            if self.state != new_state:
                self.get_logger().info(f'State changed from {self.state.name} to {new_state.name}')
                self.state = new_state
        except ValueError:
            self.get_logger().error(f'Received an invalid state value: {msg.data}')

    def scan_callback(self, scan_msg: LaserScan):
        """
        Processes LaserScan data to find the closest object and update errors.

        Args:
            scan_msg: The incoming laser scan data.
        """
        # Find the shortest distance and its corresponding angle
        # Using np.argmin is a safe way to handle potential 'inf' values in scan data
        min_dist_index = np.argmin(scan_msg.ranges)
        min_distance = scan_msg.ranges[min_dist_index]
        angle_of_min_dist = scan_msg.angle_min + min_dist_index * scan_msg.angle_increment

        # Update errors for the proportional controller
        self.distance_error = min_distance - self.TARGET_DISTANCE
        
        # RESTORED: self.angle_error is the absolute angle, matching original logic.
        self.angle_error = angle_of_min_dist
        
        # Check for obstacles using the original set of angles
        self.min_forward_distance = self.get_forward_distance(scan_msg)

    def get_forward_distance(self, scan_msg: LaserScan) -> float:
        """
        Checks for the minimum distance in a forward-facing cone using the
        original implementation's specific angles.
        """
        # RESTORED: Using the exact angles from the original code.
        angle1 = self.get_distance_at_angle(scan_msg, np.deg2rad(-180 - 10)) # -190 deg
        angle2 = self.get_distance_at_angle(scan_msg, np.deg2rad(180 - 10))  # 170 deg
        angle3 = self.get_distance_at_angle(scan_msg, np.deg2rad(180))       # 180 deg
        
        return min(angle1, angle2, angle3)

    def get_distance_at_angle(self, scan_msg: LaserScan, angle: float) -> float:
        """
        Calculates the laser scan range index for a given angle.

        Args:
            scan_msg: The laser scan data.
            angle: The desired angle in radians.

        Returns:
            The distance measurement at the specified angle.
        """
        index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
        # Ensure index is within the valid range to prevent crashes
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        return scan_msg.ranges[index]

    def timer_callback(self):
        """
        Main control loop. Publishes a 'found person' status and, if active,
        sends velocity commands to follow the person.
        """
        # Publish if a person is detected within the forward range
        found_person_msg = Bool()
        found_person_msg.data = self.min_forward_distance < self.PERSON_DETECTION_RANGE
        self.found_person_publisher.publish(found_person_msg)
        
        # Only publish velocity commands if in the correct state
        if self.state != STATE.PERSON_FOLLOW:
            return

        twist_msg = Twist()
        
        # Proportional control for linear velocity based on distance error
        twist_msg.linear.x = self.LINEAR_P_GAIN * self.distance_error
        

        # Proportional control for angular velocity based on angle error
        twist_msg.angular.z = -self.ANGULAR_P_GAIN * self.angle_error
        
        self.get_logger().debug(
            f'DistErr: {self.distance_error:.2f}, AngErr: {math.degrees(self.angle_error):.1f} | '
            f'CmdVel: (LinX: {twist_msg.linear.x:.2f}, AngZ: {twist_msg.angular.z:.2f})'
        )
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    """Initializes the ROS 2 node, spins it, and handles shutdown."""
    rclpy.init(args=args)
    node = PersonFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()