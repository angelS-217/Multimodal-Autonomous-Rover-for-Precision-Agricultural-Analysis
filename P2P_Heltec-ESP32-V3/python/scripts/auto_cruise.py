import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import random
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("启动自主探索模式...")
    time.sleep(3.0)

    while rclpy.ok():
        # 1. 核心逻辑：不再是固定的 0.5，而是在半径 1米内随机选个点
        # 这样它就会不断地向周围的“灰色地带”尝试
        target_x = random.uniform(1.5, 3.0)
        target_y = random.uniform(-0.5, 0.5)
        
        print(f"\n🧠 思考中... 决定去探索新区域: (x={target_x:.2f}, y={target_y:.2f})")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        goal_pose.pose.orientation.w = 1.0

        navigator.goal_checker_id = 'general_goal_checker'


        navigator.goToPose(goal_pose)

        # 2. 监测进度
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"正在开荒中，剩余距离: {feedback.distance_remaining:.2f} 米", end='\r')
            
            # 如果它在一个地方卡着“蠕动”超过 10 秒没进展，直接放弃，换个点
            # 这是解决“瞎动”的土办法
            time.sleep(1.0)

        print("\n✅ 到达或放弃该区域，准备寻找下一个未知点...")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
