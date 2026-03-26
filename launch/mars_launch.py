import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    tb3_share = get_package_share_directory('turtlebot3_description')
    tb3_gazebo_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')
    parent_share = os.path.dirname(tb3_share)

    urdf_file = os.path.join(tb3_share, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read().replace('${namespace}', '')

    env = os.environ.copy()
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    env['IGN_LOG_PATH'] = '/tmp/ign_logs'
    env['IGN_GAZEBO_RESOURCE_PATH'] = ':'.join([
        parent_share,
        tb3_gazebo_models,
        env.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ]).rstrip(':')

    return LaunchDescription([
        # Gazebo starten
        ExecuteProcess(
            cmd=['ign', 'gazebo',
                 os.path.expanduser(
                     '~/ros2_ws/src/embedded_systems_project/worlds/mars.sdf')],
            env=env,
            output='screen'
        ),

        # Robot Description publishen
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,       # FIX: added sim time
            }],
            output='screen'
        ),

        # Roboter in Gazebo spawnen
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3',
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'models', 'turtlebot3_burger', 'model.sdf'
                ),
                '-x', '0', '-y', '-5', '-z', '0', '-Y', '1.57'
            ],
            output='screen'
        ),

        # ROS <-> Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            ],
            parameters=[{
                'use_sim_time': True,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',  # FIX
            }],
            output='screen'
        ),
    ])