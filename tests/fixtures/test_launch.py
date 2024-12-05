"""Test launch file for integration tests"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for testing"""
    pkg_dir = get_package_share_directory('skypilot')
    
    # Declare launch arguments
    config_path = os.path.join(pkg_dir, 'tests', 'fixtures', 'test_config.yaml')
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Path to the test configuration file'
    )
    
    # Start MAVROS node
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
        }]
    )
    
    # Start Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    # Start drone controller node
    drone_controller = Node(
        package='skypilot',
        executable='drone_controller_node',
        name='drone_controller',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # Start mission controller node
    mission_controller = Node(
        package='skypilot',
        executable='mission_controller_node',
        name='mission_controller',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # Start path planner node
    path_planner = Node(
        package='skypilot',
        executable='path_planner_node',
        name='path_planner',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # Start visualization node
    visualizer = Node(
        package='skypilot',
        executable='visualization_node',
        name='visualizer',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_arg,
        mavros_node,
        gazebo,
        drone_controller,
        mission_controller,
        path_planner,
        visualizer
    ]) 