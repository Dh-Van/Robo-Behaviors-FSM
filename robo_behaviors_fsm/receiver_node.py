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
from robo_behaviors_fsm.state_machine_node import STATE

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')
        
        # hz = 0.5
        # self.timer = self.create_timer(hz, self.periodic())
        self.sub = self.create_subscription(UInt8, 'state_machine_topic', self.sub_callback, 10)
        
    def sub_callback(self, msg):
        print(STATE(msg.data))
        
def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        