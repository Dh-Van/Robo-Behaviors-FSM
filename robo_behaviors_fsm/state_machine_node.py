"""
This script implements the central state machine for controlling robot behaviors.

It orchestrates the robot's actions by transitioning between different states
based on sensor inputs. The defined behaviors include:
1. Driving in an expanding circle.
2. Following a wall.
3. Turning at a corner.
4. Driving in a straight line.
5. Following a detected person.
"""
# Standard library imports
from enum import Enum
import math

# Third-party imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8


class STATE(Enum):
    """Enumeration for the different robot operational states."""
    IDLE = 0
    ESTOP = 1
    CIRCLE = 2
    WALL_FOLLOW = 3
    TURN = 4
    STRAIGHT = 5
    PERSON_FOLLOW = 6


class StateMachineNode(Node):
    """
    Manages the robot's behavior by transitioning through a finite state machine.

    This node subscribes to various sensor topics (e.g., /estop, /wall_found)
    to make decisions and publishes the current state for other nodes to use.
    It also directly controls the robot's movement in some states (e.g., TURN).
    """
    def __init__(self):
        """Initializes the node, state, parameters, and ROS communications."""
        super().__init__('state_machine_node')

        # --- State Machine and Flags ---
        self.state = STATE.IDLE
        self.estop_triggered = False
        self.wall_is_found = False
        self.wall_has_ended = False
        self.person_is_found = False
        self.turn_start_time = None

        # --- Behavior Parameters ---
        self.TURN_ANGULAR_VELOCITY = -0.3  # rad/s (negative for clockwise)
        self.TURN_ANGLE_RAD = math.radians(135)
        self.STRAIGHT_LINEAR_VELOCITY = 0.2 # m/s

        # --- ROS 2 Communications ---
        self.state_publisher = self.create_publisher(UInt8, '/current_state', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Bool, '/estop', self.estop_callback, 10)
        self.create_subscription(Bool, '/wall_found', self.wall_found_callback, 10)
        self.create_subscription(Bool, '/wall_end', self.wall_end_callback, 10)
        self.create_subscription(Bool, '/found_person', self.found_person_callback, 10)

        self.create_timer(0.1, self.timer_callback) # 10 Hz state machine loop
        self.get_logger().info('State Machine Node has been initialized.')

    def set_state(self, new_state: STATE):
        """Updates the robot's state and logs the transition."""
        if self.state != new_state:
            self.get_logger().info(f'State changing from {self.state.name} to {new_state.name}')
            self.state = new_state

    def publish_stop_command(self):
        """Creates and publishes a Twist message with zero velocity."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def timer_callback(self):
        """The main state machine logic loop, executed at a fixed frequency."""
        # Always publish the current state for other nodes
        state_msg = UInt8()
        state_msg.data = self.state.value
        self.state_publisher.publish(state_msg)

        # --- State Transition Logic ---
        match self.state:
            case STATE.IDLE:
                self.publish_stop_command()
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)
                # TODO: Replace 'True' with a subscriber to a start signal
                elif True:
                    self.set_state(STATE.CIRCLE)

            case STATE.ESTOP:
                self.publish_stop_command()

            case STATE.CIRCLE:
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)
                elif self.wall_is_found:
                    self.set_state(STATE.WALL_FOLLOW)

            case STATE.WALL_FOLLOW:
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)
                elif self.wall_has_ended:
                    self.set_state(STATE.TURN)

            case STATE.TURN:
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)
                    return

                if self.turn_start_time is None:
                    self.turn_start_time = self.get_clock().now()

                # Command a constant turn
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.TURN_ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(twist_msg)

                # Check if turn duration is complete
                elapsed_s = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
                time_to_turn = abs(self.TURN_ANGLE_RAD / self.TURN_ANGULAR_VELOCITY)
                
                if elapsed_s > time_to_turn:
                    self.turn_start_time = None # Reset for next time
                    self.set_state(STATE.STRAIGHT)

            case STATE.STRAIGHT:
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)
                elif self.person_is_found:
                    self.set_state(STATE.PERSON_FOLLOW)
                else:
                    twist_msg = Twist()
                    twist_msg.linear.x = self.STRAIGHT_LINEAR_VELOCITY
                    self.cmd_vel_publisher.publish(twist_msg)

            case STATE.PERSON_FOLLOW:
                if self.estop_triggered:
                    self.set_state(STATE.ESTOP)

    # --- Subscriber Callbacks ---
    def estop_callback(self, msg: Bool):
        """Emergency stop callback."""
        self.estop_triggered = msg.data
        if self.estop_triggered:
            self.get_logger().warn('E-STOP TRIGGERED!')

    def wall_found_callback(self, msg: Bool):
        """Wall detection flag callback."""
        self.wall_is_found = msg.data

    def wall_end_callback(self, msg: Bool):
        """End-of-wall (corner) detection flag callback."""
        self.wall_has_ended = msg.data

    def found_person_callback(self, msg: Bool):
        """Person detection flag callback."""
        self.person_is_found = msg.data


def main(args=None):
    """Initializes the ROS 2 node, spins it, and handles shutdown."""
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()