import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('embedded_systems_project')
    world_file = os.path.join(pkg, 'worlds', 'mars.sdf')

    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Gazebo starten
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file],
            env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            output='screen'
        ),

        # ROS <-> Gazebo Bridge 
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            output='screen'
        ),

        # Robot Description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True 
            }]
        ),

        # Roboter spawnen
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'turtlebot3', '-topic', 'robot_description',
                       '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
    ])
