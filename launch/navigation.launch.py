from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_cpp',           # your package name
            executable='path_planning_module',  # your C++ node
            name='path_planning_node',          # node name in ROS 2
            output='screen',                    # logs appear in terminal
            parameters=[                        # optional: add ROS params here
                # {'param_name': value},
            ]
        )
    ])
