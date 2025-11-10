import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_armando_description = get_package_share_directory('armando_description')
    
    urdf_file = os.path.join(pkg_armando_description, 'urdf', 'armando.urdf.xacro')
    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Lancer Gazebo avec interface graphique'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation Gazebo'
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 empty.sdf'],
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'armando',
            '-z', '0.5',
            '-x', '0.0',
            '-y', '0.0',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )
    
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[load_joint_state_broadcaster]
                )
            ]
        )
    )
    
    load_position_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[load_position_controller]
                )
            ]
        )
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot_node,
        clock_bridge,
        load_joint_state_broadcaster_event,  
        load_position_controller_event,
    ])