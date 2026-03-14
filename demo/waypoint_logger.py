#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # Subscribe to the topic Foxglove publishes to when you click a goal
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  
            self.goal_callback,
            10)
            
        self.get_logger().info("📍 Waypoint Logger active. Click and drag a Nav Goal in Foxglove to generate a coordinate...")

    def goal_callback(self, msg):
        # 1. Grab the X and Y
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 2. Grab the Quaternion data
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 3. Convert Quaternion to Yaw (Theta) in radians
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # 4. Print it exactly how route.txt expects it
        print("\n--- Copy below this line ---")
        print(f"{x:.3f}, {y:.3f}, {theta:.3f}")
        print("----------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    logger_node = WaypointLogger()
    
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()