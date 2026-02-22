import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',        # 你的雷达话题
                'odom_topic': '/odom',              # 输出的里程计话题
                'publish_tf': True,                 # 必须开启！由它来发布 TF
                'base_frame_id': 'base_link',       # 你的车身坐标系
                'odom_frame_id': 'odom',            # 里程计坐标系
                'init_pose_from_topic': '',
                'freq': 20.0                        # 计算频率，20Hz 足够了
            }],
        )
    ])
