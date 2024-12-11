from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_bt_project',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
            parameters=[],
            arguments=['trees/my_tree.xml']  # XMLファイルのパス
        )
    ])
