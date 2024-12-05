"""Common test fixtures and data"""

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from mavros_msgs.msg import State

def create_test_poses():
    """Create test pose data"""
    poses = []
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = float(i)
        pose.pose.position.y = float(i)
        pose.pose.position.z = 2.0
        poses.append(pose)
    return poses

def create_test_path():
    """Create test path data"""
    path = Path()
    path.poses = create_test_poses()
    return path

def create_test_obstacle_map():
    """Create test obstacle map data"""
    obstacle_map = OccupancyGrid()
    obstacle_map.info.width = 10
    obstacle_map.info.height = 10
    obstacle_map.info.resolution = 1.0
    obstacle_map.data = [0] * 100  # Empty map
    return obstacle_map

def create_test_state():
    """Create test drone state data"""
    state = State()
    state.armed = True
    state.mode = "OFFBOARD"
    return state

def create_test_velocity():
    """Create test velocity data"""
    vel = TwistStamped()
    vel.twist.linear.x = 0.5
    vel.twist.linear.y = 0.5
    vel.twist.linear.z = 0.0
    vel.twist.angular.z = 0.1
    return vel

def create_test_mission_params():
    """Create test mission parameters"""
    return {
        'waypoint': {
            'waypoints': [
                [0.0, 0.0, 2.0],
                [1.0, 1.0, 2.0],
                [2.0, 2.0, 2.0]
            ]
        },
        'coverage': {
            'boundary_points': [
                [0.0, 0.0],
                [5.0, 0.0],
                [5.0, 5.0],
                [0.0, 5.0]
            ]
        },
        'poi': {
            'point': [2.5, 2.5, 2.0]
        }
    }

def create_test_rl_data():
    """Create test RL environment data"""
    return {
        'observation': {
            'position': np.array([1.0, 2.0, 3.0]),
            'velocity': np.array([0.5, 0.5, 0.0]),
            'goal': np.array([5.0, 5.0, 3.0]),
            'obstacles': np.zeros((10, 10))
        },
        'action': np.array([0.5, 0.5, 0.0, 0.1])
    }

def create_test_config():
    """Create test configuration data"""
    return {
        'planning': {
            'max_velocity': 2.0,
            'max_acceleration': 1.0,
            'goal_tolerance': 0.2,
            'safety_margin': 0.5
        },
        'control': {
            'position_p_gain': 1.0,
            'velocity_p_gain': 1.0,
            'max_velocity': 2.0,
            'max_yaw_rate': 1.0
        },
        'observation': {
            'position_min': [-10.0, -10.0, 0.0],
            'position_max': [10.0, 10.0, 5.0],
            'velocity_min': [-2.0, -2.0, -1.0],
            'velocity_max': [2.0, 2.0, 1.0]
        },
        'action': {
            'min_action': [-1.0, -1.0, -0.5, -1.0],
            'max_action': [1.0, 1.0, 0.5, 1.0]
        },
        'rewards': {
            'goal_reached': 100.0,
            'collision': -100.0,
            'progress': 1.0,
            'time_penalty': -0.1
        },
        'area_coverage': {
            'line_spacing': 2.0,
            'altitude': 3.0
        },
        'point_of_interest': {
            'orbit_radius': 2.0,
            'num_waypoints': 8
        }
    } 