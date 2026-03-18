import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Gazebo starten
        ExecuteProcess(
            cmd=['ign', 'gazebo', 
                 os.path.expanduser('~/ros2_ws/src/mars_world/worlds/mars.sdf')],
            env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            output='screen'
        ),
        # Robot Description publishen
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Roboter in Gazebo spawnen
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'turtlebot3', '-topic', 'robot_description',
                      '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
    ])
