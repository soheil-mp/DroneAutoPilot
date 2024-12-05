from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('drone_nav_rl')
    
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ])
    )
    
    # Spawn drone model
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'drone',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Launch MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[{
            'fcu_url': 'udp://:14540@localhost:14557',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0'
        }],
        output='screen'
    )
    
    # Launch drone controller
    drone_controller = Node(
        package='drone_nav_rl',
        executable='drone_controller',
        name='drone_controller',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        gazebo,
        spawn_drone,
        mavros_node,
        drone_controller
    ]) 