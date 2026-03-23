import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    tb3_share = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(tb3_share, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read().replace('${namespace}', '')
    env = os.environ.copy()
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    env['IGN_LOG_PATH'] = '/tmp/ign_logs'
    env['IGN_GAZEBO_RESOURCE_PATH'] = os.path.dirname(tb3_share) + (
        ':' + env['IGN_GAZEBO_RESOURCE_PATH'] if 'IGN_GAZEBO_RESOURCE_PATH' in env else ''
    )
    return LaunchDescription([
        # Gazebo starten
        ExecuteProcess(
            cmd=['ign', 'gazebo', 
                 os.path.expanduser('~/ros2_ws/src/embedded_systems_project/worlds/mars.sdf')],
            env=env,
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
