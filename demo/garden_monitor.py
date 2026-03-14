#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image       # Added for vision feed
from cv_bridge import CvBridge          # Added for ROS to OpenCV conversion
import cv2                              # Added for saving images
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time
import os
import threading

# ==============================================================================
# 1. Unified Mission & Vision Node
# ==============================================================================
class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_commander_node')
        # Default states
        self.mission_mode = "AUTO" 
        self.vision_status = "IDLE"
        self.latest_image = None
        self.bridge = CvBridge()
        
        # Subscriptions
        self.sub_mode = self.create_subscription(String, '/rover_mode', self.mode_callback, 10)
        self.sub_status = self.create_subscription(String, 'vision/status', self.status_callback, 10)
        self.sub_image = self.create_subscription(Image, 'vision/camera_feed', self.image_callback, 10)
        
        # Publishers
        self.pub_vision_cmd = self.create_publisher(String, 'vision/cmd', 10)

    def mode_callback(self, msg):
        self.mission_mode = msg.data.upper()
        if self.mission_mode == "MANUAL":
            self.get_logger().info("Mission Commander: MANUAL OVERRIDE")
        elif self.mission_mode == "AUTO":
            self.get_logger().info("Mission Commander: AUTO ENGAGED")

    def status_callback(self, msg):
        self.vision_status = msg.data

    def image_callback(self, msg):
        try:
            # Continually store the latest frame from the camera
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def send_vision_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub_vision_cmd.publish(msg)

# ==============================================================================
# 2. Waypoint Loader 
# ==============================================================================
def load_waypoints(filepath):
    waypoints = []
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            for line in f:
                clean_line = line.split('#')[0].strip()
                if not clean_line:
                    continue
                try:
                    x, y, theta = map(float, clean_line.split(','))
                    waypoints.append((x, y, theta))
                except ValueError:
                    print(f"⚠️ Skipping improperly formatted line: {line.strip()}")
    else:
        print(f"❌ Waypoint file not found at: {filepath}")
        
    return waypoints

# ==============================================================================
# 3. Main Mission Commander Loop
# ==============================================================================
def main():
    rclpy.init()

    # Launch the Mission Node in a background thread
    mission_node = MissionNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mission_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    navigator = BasicNavigator()
    
    print("⏳ Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 is active!")

    route_file = '/home/rover/ros2_ws/demo/route.txt'
    waypoints = load_waypoints(route_file)
    
    if not waypoints:
        print("🛑 No valid waypoints found. Exiting Mission Commander.")
        rclpy.shutdown()
        return

    print("\n🚀 Starting Autonomous Agricultural Patrol...\n")

    for i, (target_x, target_y, target_theta) in enumerate(waypoints):
        print(f"🚜 Navigating to Plant {i+1} (X: {target_x}, Y: {target_y}, Yaw: {target_theta})...")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        
        goal_pose.pose.orientation.z = math.sin(target_theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(target_theta / 2.0)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0

        navigator.goToPose(goal_pose)
        was_manual = False

        while not navigator.isTaskComplete():
            if mission_node.mission_mode == "MANUAL":
                if not was_manual:
                    print("\n⏸️ Manual Override Detected! Pausing Nav2...")
                    navigator.cancelTask() 
                    was_manual = True
                time.sleep(0.5)
                continue 
            
            if mission_node.mission_mode == "AUTO" and was_manual:
                print("\n▶️ Auto Re-Engaged! Recalculating path to waypoint...")
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                navigator.goToPose(goal_pose)
                was_manual = False

            time.sleep(0.5)

        # ---------------------------------------------------------
        # Check Result & Trigger Vision
        # ---------------------------------------------------------
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"🎯 Reached Plant {i+1}!")
            
            # --- START VISION SEQUENCE ---
            print("📸 Waking up Gimbal & Vision System...")
            mission_node.send_vision_cmd("START")
            
            timeout = 45 # Maximum seconds to search for the target
            start_time = time.time()
            captured = False
            
            print("⏳ Waiting for camera to raster scan and lock...")
            while (time.time() - start_time) < timeout:
                if mission_node.vision_status == "LOCKED":
                    print("🔒 Target LOCKED! Stabilizing...")
                    time.sleep(1.0) # Let the physical gimbal settle
                    
                    if mission_node.latest_image is not None:
                        # 1. Ensure the directory exists so the script doesn't crash
                        save_dir = '/home/rover/ros2_ws/photos'
                        os.makedirs(save_dir, exist_ok=True)
                        
                        # 2. Save the image with a timestamp
                        filename = f"{save_dir}/plant_{i+1}_{int(time.time())}.jpg"
                        cv2.imwrite(filename, mission_node.latest_image)
                        print(f"✅ Image saved successfully: {filename}")
                        captured = True
                    break
                
                # Check every half second while searching
                time.sleep(0.5)
                
            if not captured:
                print("⚠️ Vision system timed out or failed to lock. No image captured for this waypoint.")
                
            print("🛑 Stopping Vision System and preparing to move...")
            mission_node.send_vision_cmd("STOP")
            # --- END VISION SEQUENCE ---
            
        elif result == TaskResult.CANCELED:
            print(f"⚠️ Navigation to Plant {i+1} was canceled.")
        elif result == TaskResult.FAILED:
            print(f"❌ Navigation to Plant {i+1} failed! Moving to next target...")

    print("\n🏁 Plant health patrol complete! Returning to standby.")
    
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()