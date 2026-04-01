"""
TidyBot full simulation launch.

Localization: ground truth from Gazebo SceneBroadcaster (zero drift).
Navigation: Nav2 stack (planner + DWB controller + behavior trees).
Task: autonomous sweep-and-collect of 5 objects across two rooms.

Usage:
    source install/setup.bash
    ros2 launch tidybot simulation.launch.py

    Without autonomous navigation (manual teleop):
    ros2 launch tidybot simulation.launch.py run_nav:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tidybot')
    urdf_xacro  = PathJoinSubstitution([pkg, 'urdf', 'tidybot.urdf.xacro'])
    world_file  = PathJoinSubstitution([pkg, 'worlds', 'home.sdf'])
    map_yaml    = PathJoinSubstitution([pkg, 'maps', 'home_map.yaml'])
    nav2_params = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'tidybot.rviz'])
    robot_desc  = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])

    run_nav = DeclareLaunchArgument('run_nav', default_value='true')

    # Gazebo Fortress with the two-room home world
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file], output='screen')

    # Publish URDF joints as TF (base_footprint -> base_link -> sensors)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    # Spawn robot at Room 1 center
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'tidy_home', '-name', 'tidybot',
                   '-topic', 'robot_description',
                   '-x', '-2.0', '-y', '0.0', '-z', '0.05'],
        output='screen')

    # Bridge Gazebo <-> ROS topics
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/right_shoulder_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/right_elbow_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/right_wrist_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/world/tidy_home/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # Ground truth localization: extracts robot world pose from Gazebo's
    # SceneBroadcaster and publishes odom -> base_footprint TF at 30Hz.
    ground_truth = Node(
        package='tidybot', executable='ground_truth_tf.py',
        name='ground_truth_tf',
        parameters=[{'use_sim_time': True}],
        output='screen')

    # map = world = odom in simulation (identity transform)
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

    # Nav2 nodes: set to WARN level to suppress per-frame log spam.
    # Only real errors (FATAL) and our task node (INFO) show in terminal.
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server',
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen')

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server',
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen')

    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen')

    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen')

    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server', 'planner_server', 'controller_server',
                'bt_navigator', 'behavior_server',
            ],
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen')

    # Autonomous tidying task: sweep both rooms, collect 5 objects
    task = Node(
        package='tidybot', executable='navigate.py',
        name='tidy_task_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_nav')))

    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # Staggered startup to ensure dependencies are ready
    return LaunchDescription([
        run_nav,
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=7.0, actions=[ground_truth, map_to_odom]),
        TimerAction(period=8.0, actions=[rviz]),
        TimerAction(period=10.0, actions=[
            map_server, planner, controller, bt_nav, behaviors,
        ]),
        TimerAction(period=15.0, actions=[lifecycle]),
        TimerAction(period=30.0, actions=[task]),
    ])
