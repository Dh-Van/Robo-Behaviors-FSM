import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg._laser_scan import LaserScan

import numpy as np
import math

class LaserTestNode(Node):
    
    def __init__(self):
        super().__init__('wall_following_node')
        
        self.timer = self.create_timer(0.1, self.periodic)
        self.create_subscription(LaserScan, '/stable_scan', self.scan, 10)
        
    def scan(self, msg: LaserScan):  
        self.get_distance(msg, np.deg2rad(360))
        self.lidar_cart(msg)
        # print(f'Min angle {msg.angle_min}, Max angle {msg.angle_max}, Angle Inc {msg.angle_increment}, Min Idx {msg.ranges.index(min(msg.ranges))}')
        
    def lidar_cart(self, msg: LaserScan):
        points = []
        for i in range(0, int((abs(msg.angle_min) + abs(msg.angle_max)) / msg.angle_increment)):
            r = msg.ranges[i]
            theta = msg.angle_min + (i * msg.angle_increment)
            points.append((r * np.cos(theta), r * np.sin(theta)))
            
        return points
        
    # Angle in radians 0 -> Front, CCW
    def get_distance(self, msg, angle):
        if(angle > 2 * np.pi or angle < 0): 
            print("Invalid angle")
            angle = 0
            
        distances = msg.ranges
        # return distances[int(angle / msg.angle_increment)]
        
    def periodic(self):
       pass
        
        
def main(args=None):
    rclpy.init(args=args)
    node = LaserTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        