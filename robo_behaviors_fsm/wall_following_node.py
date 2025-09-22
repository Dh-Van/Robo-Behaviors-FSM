import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg._laser_scan import LaserScan
from robo_behaviors_fsm.state_machine_node import STATE
from geometry_msgs.msg import Twist

from pprint import pprint
import numpy as np
import math

class WallFollowingNode(Node):
    GOAL_DIST = 0.2
    P = 0.05
    
    def __init__(self):
        super().__init__('wall_following_node')
        
        self.timer = self.create_timer(0.1, self.periodic)
        
        self.state = STATE.IDLE
        self.create_subscription(UInt8, '/current_state', self.update_state, 10) # shouldn't this be /current_state?
        self.create_subscription(LaserScan, '/stable_scan', self.scan, 10)
        self.test_driver = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wall_end_pub = self.create_publisher(Bool, '/wall_end', 10)
        self.wall_found_pub = self.create_publisher(Bool, '/wall_found', 10)
        self.distance_error = 0
        self.wall_angle = -180
        self.dist_forward = 3
        self.following_time = None

        
        
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
        left_forward_dist = self.get_distance_angle(msg, np.deg2rad(-180 - 10))
        right_forward_dist = self.get_distance_angle(msg, np.deg2rad(180 - 10))
        front_forward_dist = self.get_distance_angle(msg, np.deg2rad(180))
        
        return min(left_forward_dist, (min(right_forward_dist, front_forward_dist)))
        
    def get_distance_angle(self, msg, angle):
        return msg.ranges[int((angle - msg.angle_min) / msg.angle_increment)]
            
    def periodic(self):
        found_msg = Bool()
        found_msg.data = False
        print(self.distance_error)
        if self.following_time is None:
            self.following_time = self.get_clock().now()
            
        if(abs(self.distance_error) <= 1.5):
            found_msg.data = True
        self.wall_found_pub.publish(found_msg)
            
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = (-self.P * (self.distance_error)) + (-0.05* self.wall_angle)
        
        wall_msg = Bool()
        elapsed = (self.get_clock().now() - self.following_time).nanoseconds * 1e-9
        print(self.dist_forward)
        if(elapsed > 5 and self.dist_forward <= 1.5):
            wall_msg.data = True
        else:
            wall_msg.data = False
        self.wall_end_pub.publish(wall_msg)
        
        # print(msg.angular.z)
        if(self.state == STATE.WALL_FOLLOW):
            self.test_driver.publish(msg)
            
        pass
        
        
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        