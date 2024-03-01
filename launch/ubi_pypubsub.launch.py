from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ubi_rclpy_pkg',
            executable='hellopy_publisher',
            name='hellopy_publisher',
            output='screen', 
        ),
        Node(
            package='ubi_rclpy_pkg', 
            executable='hellopy_subscriber', 
            name='hellopy_subscriber',  
            output='screen',             
        ),
    ])