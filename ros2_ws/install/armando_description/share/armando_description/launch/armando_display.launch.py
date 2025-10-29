import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Récupère le chemin du package
    pkg_path = get_package_share_directory('armando_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm.urdf')
    rviz_config = os.path.join(pkg_path, 'config', 'armando.rviz')
    
    # Lit le contenu du fichier URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])