#!/usr/bin/env python3

import unittest
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from src.utilities.path_planner import PathPlanner, PathOptimizationMethod

class TestPathPlanner(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.path_planner = PathPlanner()
        self.setup_test_data()
        
    def setup_test_data(self):
        """Create test data"""
        # Create test waypoints
        self.waypoints = []
        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = float(i)
            pose.pose.position.y = float(i)
            pose.pose.position.z = 2.0
            self.waypoints.append(pose)
            
        # Create test path
        self.test_path = Path()
        self.test_path.poses = self.waypoints
        
        # Create test obstacle map
        self.obstacle_map = OccupancyGrid()
        self.obstacle_map.info.width = 10
        self.obstacle_map.info.height = 10
        self.obstacle_map.info.resolution = 1.0
        self.obstacle_map.data = [0] * 100  # Empty map
        
    def test_shortest_path_optimization(self):
        """Test shortest path optimization"""
        points = np.array([[0, 0, 2], [1, 1, 2], [2, 2, 2], [3, 3, 2], [4, 4, 2]])
        optimized = self.path_planner.visibility_graph_optimization(points)
        
        # Should reduce waypoints while maintaining start and end
        self.assertEqual(len(optimized), 2)
        np.testing.assert_array_equal(optimized[0], points[0])
        np.testing.assert_array_equal(optimized[-1], points[-1])
        
    def test_smooth_path_optimization(self):
        """Test path smoothing"""
        original_path = self.test_path
        smoothed_path = self.path_planner.optimize_smooth_path(original_path)
        
        # Should maintain same number of points but with smoother transitions
        self.assertEqual(len(smoothed_path.poses), len(original_path.poses))
        
        # Check continuity
        for i in range(1, len(smoothed_path.poses)):
            prev_pos = smoothed_path.poses[i-1].pose.position
            curr_pos = smoothed_path.poses[i].pose.position
            
            # Calculate distance between consecutive points
            distance = np.sqrt(
                (curr_pos.x - prev_pos.x)**2 +
                (curr_pos.y - prev_pos.y)**2 +
                (curr_pos.z - prev_pos.z)**2
            )
            
            # Ensure points are not too far apart
            self.assertLess(distance, 2.0)
            
    def test_energy_efficient_path(self):
        """Test energy efficient path optimization"""
        original_path = self.test_path
        optimized_path = self.path_planner.optimize_energy_efficient_path(original_path)
        
        # Check acceleration constraints
        max_acceleration = self.path_planner.config['planning']['max_acceleration']
        
        for i in range(1, len(optimized_path.poses) - 1):
            p1 = optimized_path.poses[i-1].pose.position
            p2 = optimized_path.poses[i].pose.position
            p3 = optimized_path.poses[i+1].pose.position
            
            # Calculate acceleration
            v1 = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
            v2 = np.array([p3.x - p2.x, p3.y - p2.y, p3.z - p2.z])
            acceleration = np.linalg.norm(v2 - v1)
            
            self.assertLessEqual(acceleration, max_acceleration)
            
    def test_safe_corridor_path(self):
        """Test safe corridor path optimization"""
        # Add some obstacles to the map
        obstacle_map = self.obstacle_map
        obstacle_map.data[55] = 100  # Add obstacle
        
        self.path_planner.obstacle_map = obstacle_map
        original_path = self.test_path
        optimized_path = self.path_planner.optimize_safe_corridor_path(original_path)
        
        # Check safety margin
        safety_margin = self.path_planner.config['obstacle_avoidance']['safety_margin']
        
        for pose in optimized_path.poses:
            x = int(pose.pose.position.x)
            y = int(pose.pose.position.y)
            
            # Check if point is within map bounds
            if (0 <= x < obstacle_map.info.width and 
                0 <= y < obstacle_map.info.height):
                # Get surrounding cells
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        check_x = x + dx
                        check_y = y + dy
                        if (0 <= check_x < obstacle_map.info.width and 
                            0 <= check_y < obstacle_map.info.height):
                            idx = check_y * obstacle_map.info.width + check_x
                            # Ensure no obstacles within safety margin
                            self.assertLess(obstacle_map.data[idx], 50)
                            
    def test_line_of_sight(self):
        """Test line of sight checking"""
        # Create test points
        start = np.array([0, 0, 2])
        end = np.array([5, 5, 2])
        
        # Test with empty map
        self.assertTrue(
            self.path_planner.has_line_of_sight(start, end)
        )
        
        # Test with obstacle
        obstacle_map = self.obstacle_map
        obstacle_map.data[55] = 100  # Add obstacle in the middle
        self.path_planner.obstacle_map = obstacle_map
        
        self.assertFalse(
            self.path_planner.has_line_of_sight(start, end)
        )
        
    def test_world_to_map_conversion(self):
        """Test world to map coordinate conversion"""
        # Test point conversion
        world_point = np.array([2.5, 1.5, 0])
        map_x, map_y = self.path_planner.world_to_map(
            world_point,
            self.obstacle_map.info.resolution,
            self.obstacle_map.info.origin
        )
        
        self.assertEqual(map_x, 2)
        self.assertEqual(map_y, 1)
        
    def test_is_valid_point(self):
        """Test point validity checking"""
        # Test valid point
        self.assertTrue(self.path_planner.is_valid_point(5, 5))
        
        # Test out of bounds
        self.assertFalse(self.path_planner.is_valid_point(-1, 5))
        self.assertFalse(self.path_planner.is_valid_point(5, 10))
        
        # Test with obstacle
        self.obstacle_map.data[55] = 100
        self.path_planner.obstacle_map = self.obstacle_map
        self.assertFalse(self.path_planner.is_valid_point(5, 5))

if __name__ == '__main__':
    unittest.main() 