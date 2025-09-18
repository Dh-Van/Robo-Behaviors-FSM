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
        hz = 3
        
        self.state_pub = self.create_publisher(UInt8, '/current_state', 10)
        self.timer = self.create_timer(hz, self.periodic)
        self.i = 0
        self.estop = self.create_subscription(erm, '/estop', 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        
    def periodic(self):
        self.i += 1
        msg = UInt8()
        msg.data = STATE(self.i % 6).value
        self.state_pub.publish(msg)


        # match(self.state):
        #     case STATE.IDLE:
        #         # if you get a message saying that you may start
        #         if True: # REPLACE TRUE WITH SUB
        #             self.state = STATE.CIRCLE
        #         if self.estop:
        #             self.state = STATE.ESTOP
        #     case STATE.ESTOP:
        #         # TELL MOTOR TO STOP
        #     case STATE.CIRCLE:
                
        #         self.state_pub
        
def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
        
        