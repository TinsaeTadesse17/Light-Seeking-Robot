from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('light_seeking_robot'),
        'worlds',
        'light_seeking_world.sdf'
    )

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # ROS-Gazebo Bridge
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist]',
                '--ros-args', '-p', 'direction:=1'
            ],
            output='screen'
        ),

        # Existing nodes
        Node(
            package='light_seeking_robot',
            executable='light_seeking_node',
            output='screen'
        ),
    
    ])