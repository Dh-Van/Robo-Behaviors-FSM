import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg._laser_scan import LaserScan
from robo_behaviors_fsm.state_machine_node import STATE
from geometry_msgs.msg import Twist

from pprint import pprint
import numpy as np
import math

class WallFollowingNode(Node):
    GOAL_DIST = 0.5
    P = 0.3
    
    def __init__(self):
        super().__init__('wall_following_node')
        
        self.timer = self.create_timer(0.1, self.periodic)
        
        self.state = STATE.IDLE
        self.create_subscription(UInt8, 'state_machine_topic', self.update_state, 10) # shouldn't this be /current_state?
        self.create_subscription(LaserScan, '/stable_scan', self.scan, 10)
        self.test_driver = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.distance_error = 0
        self.wall_angle = -180
        self.dist_forward = 3
        
        
    def update_state(self, msg):
        try:
            self.state = STATE(msg.data)
        except ValueError as e:
            print(f'ERROR: Invalid State - {e}')
            
    def scan(self, msg: LaserScan):        
        # print(f'LEFT: {self.get_distance_angle(msg, np.deg2rad(-90))}\tRIGHT: {self.get_distance_angle(msg, np.deg2rad(90))}')
        dist_a = self.get_distance_angle(msg, np.deg2rad(-90))
        dist_b = self.get_distance_angle(msg, np.deg2rad(-135))
        
        self.dist_forward = self.get_forward_distance(msg)
        
        point_a = (dist_a * math.cos(np.deg2rad(-90)), dist_a * math.sin(np.deg2rad(-90)))
        point_b = (dist_b * math.cos(np.deg2rad(-135)), dist_b * math.sin(np.deg2rad(-135)))
        
        self.wall_angle = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        
        self.distance_error = self.GOAL_DIST - dist_a
        
    def get_forward_distance(self, msg):
        left_forward_dist = self.get_distance_angle(msg, np.deg2rad(-180 - 20))
        right_forward_dist = self.get_distance_angle(msg, np.deg2rad(180 - 20))
        front_forward_dist = self.get_distance_angle(msg, np.deg2rad(180))
        
        return min(left_forward_dist, (min(right_forward_dist, front_forward_dist)))
        
    def get_distance_angle(self, msg, angle):
        return msg.ranges[int((angle - msg.angle_min) / msg.angle_increment)]
            
    def periodic(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = (-self.P * (self.distance_error)) + (-0.1 * self.wall_angle)
        
        if(self.dist_forward <= self.GOAL_DIST * 2):
            msg.angular.z = -1.0
        
        print(msg.angular.z)
        self.test_driver.publish(msg)
        # if(self.state != STATE.WALL_FOLLOW):
        #     return
        pass
        
        
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        