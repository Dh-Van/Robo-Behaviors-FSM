"""
This script implements a ROS 2 node for a wall-following behavior.

It uses LaserScan data to calculate the distance and angle to a nearby wall,
then uses a proportional controller to maintain a fixed distance from it. It
also publishes status messages indicating when a wall is found or when a
corner is detected.
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


class WallFollowingNode(Node):
    """
    Navigates the robot alongside a wall using a proportional controller.

    This node calculates the wall's angle and the robot's distance from it
    using two points from a laser scan. It publishes velocity commands to
    correct the robot's heading and distance, keeping it parallel to the wall.
    """
    def __init__(self):
        """Initializes the node, parameters, and ROS communications."""
        super().__init__('wall_following_node')

        # --- Control & Behavior Parameters ---
        self.TARGET_DISTANCE = 0.2              # Target distance from the wall (meters)
        self.LINEAR_VELOCITY = 0.1              # Constant forward speed while following (m/s)
        self.DISTANCE_P_GAIN = 0.05             # Gain for distance error correction
        self.ANGLE_P_GAIN = 0.05                # Gain for wall angle error correction
        self.PRIMARY_SCAN_ANGLE = math.radians(-90)   # Angle for primary distance measurement
        self.SECONDARY_SCAN_ANGLE = math.radians(-135) # Angle for calculating wall orientation
        self.WALL_DETECTION_THRESHOLD = 1.5     # Max distance to consider a wall "found"
        self.CORNER_TIMEOUT_S = 5.0             # Time before corner detection is active
        self.CORNER_DISTANCE_THRESHOLD = 1.5    # Obstacle distance to trigger a corner event

        # --- Internal State Variables ---
        self.state = STATE.IDLE
        self.distance_error = float('inf')
        self.wall_angle_rad = 0.0
        self.forward_distance = float('inf')
        self.following_start_time = None

        # --- ROS 2 Communications ---
        self.create_subscription(UInt8, '/current_state', self.state_callback, 10)
        self.create_subscription(LaserScan, '/stable_scan', self.scan_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wall_end_publisher = self.create_publisher(Bool, '/wall_end', 10)
        self.wall_found_publisher = self.create_publisher(Bool, '/wall_found', 10)
        self.create_timer(0.1, self.timer_callback) # 10 Hz control loop

        self.get_logger().info('Wall Following Node has been initialized.')

    def state_callback(self, msg: UInt8):
        """Updates the node's internal state from the /current_state topic."""
        try:
            new_state = STATE(msg.data)
            if self.state != new_state and new_state == STATE.WALL_FOLLOW:
                # Reset timer when entering WALL_FOLLOW state
                self.following_start_time = self.get_clock().now()
            self.state = new_state
        except ValueError:
            self.get_logger().error(f'Received an invalid state value: {msg.data}')

    def scan_callback(self, scan_msg: LaserScan):
        """
        Processes LaserScan data to calculate wall angle and distance error.

        This method samples two points from the laser scan (at -90 and -135
        degrees) to form a line segment representing the wall. It then
        calculates the angle of this segment and the error between the robot's
        current distance and the target distance.
        """
        # Get distances at two fixed angles to define the wall segment
        dist_a = self.get_distance_at_angle(scan_msg, self.PRIMARY_SCAN_ANGLE)
        dist_b = self.get_distance_at_angle(scan_msg, self.SECONDARY_SCAN_ANGLE)

        # Convert polar coordinates (distance, angle) to Cartesian (x, y)
        point_a = (dist_a * math.cos(self.PRIMARY_SCAN_ANGLE), dist_a * math.sin(self.PRIMARY_SCAN_ANGLE))
        point_b = (dist_b * math.cos(self.SECONDARY_SCAN_ANGLE), dist_b * math.sin(self.SECONDARY_SCAN_ANGLE))

        # Calculate the angle of the wall relative to the robot's frame
        self.wall_angle_rad = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        
        # Calculate the error from the target distance to the wall
        self.distance_error = self.TARGET_DISTANCE - dist_a
        
        # Check for obstacles directly in front
        self.forward_distance = self.get_forward_distance(scan_msg)

    def get_forward_distance(self, scan_msg: LaserScan) -> float:
        """Checks for the minimum distance in a forward-facing cone."""
        center_dist = self.get_distance_at_angle(scan_msg, np.deg2rad(180))
        left_dist = self.get_distance_at_angle(scan_msg, np.deg2rad(180 - 10))
        right_dist = self.get_distance_at_angle(scan_msg, np.deg2rad(180 + 10))
        return min(center_dist, left_dist, right_dist)

    def get_distance_at_angle(self, scan_msg: LaserScan, angle: float) -> float:
        """Calculates the laser scan range index for a given angle."""
        index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
        # Clamp index to the valid range to prevent crashes
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        return scan_msg.ranges[index]

    def timer_callback(self):
        """Main control loop for wall following logic."""
        # --- Publish Status Messages ---
        wall_found_msg = Bool()
        wall_found_msg.data = abs(self.distance_error) <= self.WALL_DETECTION_THRESHOLD
        self.wall_found_publisher.publish(wall_found_msg)

        wall_end_msg = Bool()
        # A "wall end" or corner is detected if enough time has passed AND
        # an obstacle is seen close ahead.
        if self.following_start_time:
            elapsed_s = (self.get_clock().now() - self.following_start_time).nanoseconds * 1e-9
            if elapsed_s > self.CORNER_TIMEOUT_S and self.forward_distance <= self.CORNER_DISTANCE_THRESHOLD:
                wall_end_msg.data = True
            else:
                wall_end_msg.data = False
        else:
            wall_end_msg.data = False
        self.wall_end_publisher.publish(wall_end_msg)

        # --- Execute Control Logic ---
        if self.state == STATE.WALL_FOLLOW:
            twist_msg = Twist()
            twist_msg.linear.x = self.LINEAR_VELOCITY

            # P-Controller for angular velocity: corrects for distance and angle errors.
            distance_correction = self.DISTANCE_P_GAIN * self.distance_error
            angle_correction = self.ANGLE_P_GAIN * self.wall_angle_rad
            twist_msg.angular.z = -distance_correction - angle_correction

            self.get_logger().debug(
                f'DistErr: {self.distance_error:.2f}, WallAng: {math.degrees(self.wall_angle_rad):.1f} | '
                f'CmdVel AngZ: {twist_msg.angular.z:.2f}'
            )
            self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    """Initializes the ROS 2 node, spins it, and handles shutdown."""
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()