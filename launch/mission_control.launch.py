from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('drone_nav_rl')
    
    # Declare launch arguments
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(pkg_dir, 'config'))
    
    # Load mission controller node
    mission_controller = Node(
        package='drone_nav_rl',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'config_dir': config_dir,
            'use_sim_time': True
        }]
    )
    
    # Load path planner node
    path_planner = Node(
        package='drone_nav_rl',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'config_dir': config_dir,
            'use_sim_time': True
        }]
    )
    
    # Load mission visualization node
    mission_viz = Node(
        package='drone_nav_rl',
        executable='mission_visualization',
        name='mission_visualization',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    
    # Return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(pkg_dir, 'config'),
            description='Directory for config files'
        ),
        mission_controller,
        path_planner,
        mission_viz
    ]) 