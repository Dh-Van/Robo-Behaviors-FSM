import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg._laser_scan import LaserScan
from robo_behaviors_fsm.state_machine_node import STATE
from geometry_msgs.msg import Twist

from pprint import pprint
import numpy as np
import math

class PersonFollowingNode(Node):
    def __init__(self):
        super().__init__('person_following_node')
        
        self.timer = self.create_timer(0.25, self.periodic)
        
        self.state = STATE.IDLE
        self.create_subscription(UInt8, '/current_state', self.update_state, 10) 
        self.create_subscription(LaserScan, '/stable_scan', self.scan, 10)
        self.found_person_pub = self.create_publisher(Bool, '/found_person', 10)
        self.test_driver = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.distance_error = 0
        self.angle_error = 0
        self.forward = 1

        
        
    def update_state(self, msg):
        try:
            self.state = STATE(msg.data)
        except ValueError as e:
            print(f'ERROR: Invalid State - {e}')
            
    def scan(self, msg: LaserScan):
        detection_distances = msg.ranges
            
        min_distance = min(detection_distances)
        angle = msg.angle_min + (detection_distances.index(min_distance)) * msg.angle_increment

        # print(angle)
        # print(f"{min_distance=}")
        self.distance_error = min_distance - 0.7
        
        self.forward = self.get_forward_distance(msg)
        self.angle_error = angle

    
    def get_forward_distance(self, msg):
        left_forward_dist = self.get_distance_angle(msg, np.deg2rad(-180 - 10))
        right_forward_dist = self.get_distance_angle(msg, np.deg2rad(180 - 10))
        front_forward_dist = self.get_distance_angle(msg, np.deg2rad(180))
        
        return min(left_forward_dist, (min(right_forward_dist, front_forward_dist)))
        
    def get_distance_angle(self, msg, angle):
        return msg.ranges[int((angle - msg.angle_min) / msg.angle_increment)]
                                
    def periodic(self):
        msg = Twist()
        found_person = Bool()
        found_person.data = False
        if self.forward  < 2:
            found_person.data = True
        self.found_person_pub.publish(found_person)
        
        msg.linear.x = (0.1 * self.distance_error)  ## this should take angle into account so for large angle errors 
        if abs(self.angle_error) > math.radians(45):
            msg.linear.x = 0.0
        print(f'{self.distance_error}, LinearX: {msg.linear.x}')

        msg.angular.z = 0.1 * -self.angle_error
        if(self.state == STATE.PERSON_FOLLOW):
            self.test_driver.publish(msg)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        