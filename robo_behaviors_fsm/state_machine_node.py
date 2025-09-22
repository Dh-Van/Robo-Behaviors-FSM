'''
Behaviors:
1) Drive in an increasingly big circle
2) If a wall is seen, follow the wall
3) If at a wall corner, turn 135deg
4) Drive in a straight line
5) If a person is seen, follow the person at a set distance
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

from enum import Enum, auto

class STATE(Enum):
    IDLE = 0
    ESTOP = 1
    CIRCLE = 2
    WALL_FOLLOW = 3
    TURN = 4
    STRAIGHT = 5
    PERSON_FOLLOW = 6

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        self.state = STATE.IDLE
        hz = 3 # 1/3?
        
        self.state_pub = self.create_publisher(String, '/current_state', 10)
        self.timer = self.create_timer(hz, self.periodic)
        self.i = 0
        self.estop = False
        self.estop_sub = self.create_subscription(Bool, '/estop', self.estop_callback, 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wall_found_sub = self.create_subscription(Bool, '/wall_found', self.wall_found_callback, 10)
        self.wall_found = False
        self.wall_end_sub = self.create_subscription(Bool, '/wall_end', self.wall_end_callback, 10)
        self.wall_end = False
        self.turn_start_time = None
        self.found_person_sub = self.create_subscription(Bool, '/found_person', self.found_person_callback, 10)

        self.found_person = False
    def periodic(self):
        # self.i += 1
        # msg = UInt8()
        # msg.data = STATE(self.i % 6).value
        # msg.data = STATE.CIRCLE.value
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

        match(self.state):
            case STATE.IDLE:
                # if you get a message saying that you may start
                if True: # REPLACE TRUE WITH SUB
                    self.state = STATE.CIRCLE
                if self.estop:
                    self.state = STATE.ESTOP
            case STATE.ESTOP:
                # TELL MOTOR TO STOP
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.twist_pub.publish(twist_msg)
            case STATE.CIRCLE:
                if self.estop:
                    self.state = STATE.ESTOP
                if self.wall_found:
                    self.state = STATE.WALL_FOLLOW

            case STATE.WALL_FOLLOW:
                # follow the wall How to tell it to start???
                if self.estop:
                    self.state = STATE.ESTOP

                if self.wall_end:
                    self.state = STATE.TURN

            case STATE.TURN:
                if self.estop:
                    self.state = STATE.ESTOP
                if self.turn_start_time is None:
                    self.turn_start_time = self.get_clock().now()
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.3
                elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
                time_to_turn = 2 #idk math
                if(elapsed > time_to_turn):
                    self.state = STATE.STRAIGHT
                    self.turn_start_time = None
            case STATE.STRAIGHT:
                if self.estop:
                    self.state = STATE.ESTOP

                twist_msg = Twist()
                twist_msg.linear.x = 0.5
                if self.found_person:
                    self.state = STATE.PERSON_FOLLOW

                
            case STATE.PERSON_FOLLOW:
                if self.estop:
                    self.state = STATE.ESTOP
                

    def wall_found_callback(self, msg):
        self.wall_found = msg.data

    def wall_end_callback(self, msg):
        self.wall_end = msg.data

    def estop_callback(self, msg):
        self.estop = msg.data
        
    def found_person_callback(self, msg):
        self.found_person = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
        
        