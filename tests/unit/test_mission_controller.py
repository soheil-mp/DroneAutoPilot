#!/usr/bin/env python3

import unittest
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from src.utilities.mission_controller import MissionController, MissionState

class TestMissionController(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.mission_controller = MissionController()
        self.setup_test_data()
        
    def setup_test_data(self):
        """Create test data"""
        # Create test waypoints
        self.waypoints = []
        for i in range(3):
            pose = PoseStamped()
            pose.pose.position.x = float(i)
            pose.pose.position.y = float(i)
            pose.pose.position.z = 2.0
            self.waypoints.append(pose)
            
        # Create test mission parameters
        self.waypoint_params = {
            'waypoints': [
                [0.0, 0.0, 2.0],
                [1.0, 1.0, 2.0],
                [2.0, 2.0, 2.0]
            ]
        }
        
        self.coverage_params = {
            'boundary_points': [
                [0.0, 0.0],
                [5.0, 0.0],
                [5.0, 5.0],
                [0.0, 5.0]
            ]
        }
        
        self.poi_params = {
            'point': [2.5, 2.5, 2.0]
        }
        
    def test_waypoint_mission_generation(self):
        """Test waypoint mission generation"""
        waypoints = self.mission_controller.generate_waypoint_mission(
            self.waypoint_params
        )
        
        # Check number of waypoints
        self.assertEqual(len(waypoints), 3)
        
        # Check waypoint positions
        for i, waypoint in enumerate(waypoints):
            self.assertEqual(waypoint.pose.position.x, float(i))
            self.assertEqual(waypoint.pose.position.y, float(i))
            self.assertEqual(waypoint.pose.position.z, 2.0)
            
    def test_coverage_mission_generation(self):
        """Test area coverage mission generation"""
        waypoints = self.mission_controller.generate_coverage_mission(
            self.coverage_params
        )
        
        # Check that waypoints cover the area
        x_coords = [wp.pose.position.x for wp in waypoints]
        y_coords = [wp.pose.position.y for wp in waypoints]
        
        # Check bounds
        self.assertGreaterEqual(min(x_coords), 0.0)
        self.assertLessEqual(max(x_coords), 5.0)
        self.assertGreaterEqual(min(y_coords), 0.0)
        self.assertLessEqual(max(y_coords), 5.0)
        
        # Check spacing
        line_spacing = self.mission_controller.config['area_coverage']['line_spacing']
        for i in range(1, len(waypoints)):
            dist = np.sqrt(
                (waypoints[i].pose.position.x - waypoints[i-1].pose.position.x)**2 +
                (waypoints[i].pose.position.y - waypoints[i-1].pose.position.y)**2
            )
            self.assertLessEqual(dist, line_spacing * 1.5)
            
    def test_perimeter_mission_generation(self):
        """Test perimeter inspection mission generation"""
        waypoints = self.mission_controller.generate_perimeter_mission({
            'boundary_points': self.coverage_params['boundary_points']
        })
        
        # Check that waypoints form a closed loop
        self.assertEqual(len(waypoints), len(self.coverage_params['boundary_points']) + 1)
        
        # Check first and last points match
        first = waypoints[0].pose.position
        last = waypoints[-1].pose.position
        self.assertEqual(first.x, last.x)
        self.assertEqual(first.y, last.y)
        self.assertEqual(first.z, last.z)
        
    def test_poi_mission_generation(self):
        """Test point of interest mission generation"""
        waypoints = self.mission_controller.generate_poi_mission(
            self.poi_params
        )
        
        # Check that waypoints form a circle around POI
        center_x = self.poi_params['point'][0]
        center_y = self.poi_params['point'][1]
        orbit_radius = self.mission_controller.config['point_of_interest']['orbit_radius']
        
        for waypoint in waypoints:
            # Calculate distance from POI
            dist = np.sqrt(
                (waypoint.pose.position.x - center_x)**2 +
                (waypoint.pose.position.y - center_y)**2
            )
            self.assertAlmostEqual(dist, orbit_radius, places=2)
            
    def test_mission_state_transitions(self):
        """Test mission state transitions"""
        # Initial state should be IDLE
        self.assertEqual(self.mission_controller.current_state, MissionState.IDLE)
        
        # Test loading mission
        self.mission_controller.current_state = MissionState.LOADING
        self.assertEqual(self.mission_controller.current_state, MissionState.LOADING)
        
        # Test executing mission
        self.mission_controller.current_state = MissionState.EXECUTING
        self.assertEqual(self.mission_controller.current_state, MissionState.EXECUTING)
        
        # Test pausing mission
        self.mission_controller.current_state = MissionState.PAUSED
        self.assertEqual(self.mission_controller.current_state, MissionState.PAUSED)
        
        # Test completing mission
        self.mission_controller.current_state = MissionState.COMPLETED
        self.assertEqual(self.mission_controller.current_state, MissionState.COMPLETED)
        
        # Test aborting mission
        self.mission_controller.current_state = MissionState.ABORTED
        self.assertEqual(self.mission_controller.current_state, MissionState.ABORTED)
        
    def test_waypoint_completion(self):
        """Test waypoint completion logic"""
        # Load test mission
        self.mission_controller.mission_waypoints = self.waypoints
        self.mission_controller.current_waypoint_index = 0
        
        # Test waypoint not reached
        drone_pose = PoseStamped()
        drone_pose.pose.position.x = 0.1
        drone_pose.pose.position.y = 0.1
        drone_pose.pose.position.z = 2.0
        
        self.mission_controller.pose_callback(drone_pose)
        self.assertEqual(self.mission_controller.current_waypoint_index, 0)
        
        # Test waypoint reached
        drone_pose.pose.position.x = 0.0
        drone_pose.pose.position.y = 0.0
        
        self.mission_controller.pose_callback(drone_pose)
        self.assertEqual(self.mission_controller.current_waypoint_index, 1)
        
    def test_mission_progress_calculation(self):
        """Test mission progress calculation"""
        # Load test mission
        self.mission_controller.mission_waypoints = self.waypoints
        self.mission_controller.current_waypoint_index = 1
        
        # Calculate progress
        progress = self.mission_controller.calculate_mission_progress()
        self.assertAlmostEqual(progress, 0.333, places=3)
        
        # Test completed mission
        self.mission_controller.current_waypoint_index = len(self.waypoints)
        progress = self.mission_controller.calculate_mission_progress()
        self.assertEqual(progress, 1.0)

if __name__ == '__main__':
    unittest.main() 