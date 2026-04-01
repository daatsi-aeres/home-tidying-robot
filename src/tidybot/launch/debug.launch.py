"""
TidyBot debug launch — teleop mode, no Nav2.

Starts Gazebo, ground truth localization, map server, RViz, and
keyboard teleop. Use this to verify sensor data, TF chain, and
map alignment before running the full simulation.

Usage:
    source install/setup.bash
    ros2 launch tidybot debug.launch.py

    Controls (in the xterm window):
      i/,  = forward/backward
      j/l  = turn left/right
      k    = stop
      z/x  = decrease/increase speed
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tidybot')
    urdf_xacro = PathJoinSubstitution([pkg, 'urdf', 'tidybot.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg, 'worlds', 'home.sdf'])
    map_yaml   = PathJoinSubstitution([pkg, 'maps', 'home_map.yaml'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'tidybot.rviz'])
    robot_desc = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file], output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'tidy_home', '-name', 'tidybot',
                   '-topic', 'robot_description',
                   '-x', '-2.0', '-y', '0.0', '-z', '0.05'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/world/tidy_home/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    ground_truth = Node(
        package='tidybot', executable='ground_truth_tf.py',
        name='ground_truth_tf',
        parameters=[{'use_sim_time': True}],
        output='screen')

    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
        output='screen')

    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_debug',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        output='screen')

    teleop = Node(
        package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
        name='teleop',
        parameters=[{'use_sim_time': True}],
        prefix='xterm -e',
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=7.0, actions=[ground_truth, map_to_odom]),
        TimerAction(period=8.0, actions=[map_server]),
        TimerAction(period=10.0, actions=[lifecycle]),
        TimerAction(period=12.0, actions=[rviz]),
        TimerAction(period=14.0, actions=[teleop]),
    ])
