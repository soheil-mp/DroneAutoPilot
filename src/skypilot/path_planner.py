#!/usr/bin/env python3

import os
import yaml
import numpy as np
from enum import Enum
from typing import List, Dict, Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from drone_nav_rl_interfaces.srv import OptimizePath

class PathOptimizationMethod(Enum):
    SHORTEST_PATH = "shortest_path"
    SMOOTHING = "smoothing"
    ENERGY_EFFICIENT = "energy_efficient"
    SAFE_CORRIDOR = "safe_corridor"

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Load configuration
        self.config = self.load_config()
        
        # Initialize state
        self.obstacle_map = None
        self.start_pose = None
        self.goal_pose = None
        
        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/planning/visualization',
            10
        )
        
        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Create services
        self.optimize_path_srv = self.create_service(
            OptimizePath,
            '/planning/optimize_path',
            self.optimize_path_callback
        )
        
    def load_config(self) -> Dict:
        """Load configuration from yaml file"""
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../config/navigation_params.yaml'
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)['navigation']
    
    def map_callback(self, msg: OccupancyGrid):
        """Store and process occupancy grid map"""
        self.obstacle_map = msg
        
    def pose_callback(self, msg: PoseStamped):
        """Update current drone pose"""
        self.start_pose = msg
        
    def optimize_path_callback(self, request, response):
        """Handle path optimization service requests"""
        try:
            # Convert waypoint array to path
            original_path = self.waypoints_to_path(request.waypoints)
            
            # Optimize path based on method
            if request.optimization_method == PathOptimizationMethod.SHORTEST_PATH.value:
                optimized_path = self.optimize_shortest_path(original_path)
            elif request.optimization_method == PathOptimizationMethod.SMOOTHING.value:
                optimized_path = self.optimize_smooth_path(original_path)
            elif request.optimization_method == PathOptimizationMethod.ENERGY_EFFICIENT.value:
                optimized_path = self.optimize_energy_efficient_path(original_path)
            elif request.optimization_method == PathOptimizationMethod.SAFE_CORRIDOR.value:
                optimized_path = self.optimize_safe_corridor_path(original_path)
            else:
                raise ValueError(f"Unknown optimization method: {request.optimization_method}")
            
            # Publish visualization
            self.publish_path_visualization(optimized_path)
            
            response.success = True
            response.optimized_path = optimized_path
            response.message = "Path optimization successful"
            
        except Exception as e:
            response.success = False
            response.message = f"Path optimization failed: {str(e)}"
        
        return response
    
    def optimize_shortest_path(self, path: Path) -> Path:
        """Optimize path for shortest distance"""
        # Convert path to numpy array for easier processing
        points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                          for pose in path.poses])
        
        # Apply visibility graph algorithm
        optimized_points = self.visibility_graph_optimization(points)
        
        # Convert back to Path message
        return self.points_to_path(optimized_points)
    
    def optimize_smooth_path(self, path: Path) -> Path:
        """Optimize path for smoothness using cubic spline interpolation"""
        points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                          for pose in path.poses])
        
        # Parameter for path smoothing
        smoothing_factor = 0.1
        
        # Apply cubic spline interpolation
        t = np.linspace(0, 1, len(points))
        t_new = np.linspace(0, 1, len(points) * 5)
        
        # Smooth each dimension separately
        smoothed_points = []
        for dim in range(3):
            from scipy.interpolate import UnivariateSpline
            spline = UnivariateSpline(t, points[:, dim], s=smoothing_factor)
            smoothed_points.append(spline(t_new))
        
        smoothed_points = np.array(smoothed_points).T
        return self.points_to_path(smoothed_points)
    
    def optimize_energy_efficient_path(self, path: Path) -> Path:
        """Optimize path for energy efficiency"""
        points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                          for pose in path.poses])
        
        # Consider drone dynamics and energy model
        max_acceleration = self.config['planning']['max_acceleration']
        max_velocity = self.config['planning']['max_velocity']
        
        # Apply energy-efficient trajectory optimization
        optimized_points = self.minimize_energy_consumption(
            points,
            max_acceleration,
            max_velocity
        )
        
        return self.points_to_path(optimized_points)
    
    def optimize_safe_corridor_path(self, path: Path) -> Path:
        """Optimize path with safety corridors around obstacles"""
        points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                          for pose in path.poses])
        
        # Get safety parameters
        safety_margin = self.config['obstacle_avoidance']['safety_margin']
        
        # Generate safety corridors
        corridors = self.generate_safety_corridors(points, safety_margin)
        
        # Optimize path within corridors
        optimized_points = self.optimize_within_corridors(points, corridors)
        
        return self.points_to_path(optimized_points)
    
    def visibility_graph_optimization(self, points: np.ndarray) -> np.ndarray:
        """Implement visibility graph path optimization"""
        # Simple implementation - connect points if line of sight exists
        optimized_points = []
        current_point = points[0]
        optimized_points.append(current_point)
        
        i = 0
        while i < len(points) - 1:
            # Look ahead for furthest visible point
            for j in range(len(points) - 1, i, -1):
                if self.has_line_of_sight(current_point, points[j]):
                    current_point = points[j]
                    optimized_points.append(current_point)
                    i = j
                    break
            i += 1
        
        return np.array(optimized_points)
    
    def minimize_energy_consumption(
        self,
        points: np.ndarray,
        max_acceleration: float,
        max_velocity: float
    ) -> np.ndarray:
        """Optimize trajectory for minimum energy consumption"""
        # Simplified energy model - minimize acceleration changes
        optimized_points = []
        
        for i in range(len(points)):
            if i == 0 or i == len(points) - 1:
                optimized_points.append(points[i])
                continue
                
            # Calculate acceleration
            prev_vel = points[i] - points[i-1]
            next_vel = points[i+1] - points[i]
            
            # Minimize acceleration change
            avg_vel = (prev_vel + next_vel) / 2
            avg_vel_norm = np.linalg.norm(avg_vel)
            
            if avg_vel_norm > max_velocity:
                avg_vel = avg_vel * max_velocity / avg_vel_norm
                
            new_point = points[i-1] + avg_vel
            optimized_points.append(new_point)
        
        return np.array(optimized_points)
    
    def generate_safety_corridors(
        self,
        points: np.ndarray,
        safety_margin: float
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Generate safety corridors around the path"""
        corridors = []
        
        for i in range(len(points) - 1):
            # Calculate corridor bounds
            direction = points[i+1] - points[i]
            direction_norm = np.linalg.norm(direction)
            if direction_norm < 1e-6:
                continue
                
            direction = direction / direction_norm
            perpendicular = np.array([-direction[1], direction[0], 0])
            
            # Create corridor bounds
            lower_bound = points[i] - perpendicular * safety_margin
            upper_bound = points[i] + perpendicular * safety_margin
            
            corridors.append((lower_bound, upper_bound))
        
        return corridors
    
    def optimize_within_corridors(
        self,
        points: np.ndarray,
        corridors: List[Tuple[np.ndarray, np.ndarray]]
    ) -> np.ndarray:
        """Optimize path while staying within safety corridors"""
        optimized_points = [points[0]]
        
        for i in range(1, len(points) - 1):
            lower_bound, upper_bound = corridors[i-1]
            
            # Project point onto corridor
            direction = points[i+1] - points[i-1]
            direction_norm = np.linalg.norm(direction)
            if direction_norm < 1e-6:
                optimized_points.append(points[i])
                continue
                
            direction = direction / direction_norm
            
            # Find closest point in corridor
            projected = points[i-1] + np.dot(points[i] - points[i-1], direction) * direction
            projected = np.clip(projected, lower_bound, upper_bound)
            
            optimized_points.append(projected)
        
        optimized_points.append(points[-1])
        return np.array(optimized_points)
    
    def has_line_of_sight(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Check if there is a clear line of sight between two points"""
        if self.obstacle_map is None:
            return True
            
        # Convert points to map coordinates
        resolution = self.obstacle_map.info.resolution
        origin = self.obstacle_map.info.origin
        
        start_map = self.world_to_map(start, resolution, origin)
        end_map = self.world_to_map(end, resolution, origin)
        
        # Use Bresenham's line algorithm to check for obstacles
        x0, y0 = start_map
        x1, y1 = end_map
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        
        for _ in range(n):
            if not self.is_valid_point(x, y):
                return False
                
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        
        return True
    
    def world_to_map(
        self,
        point: np.ndarray,
        resolution: float,
        origin: Point
    ) -> Tuple[int, int]:
        """Convert world coordinates to map coordinates"""
        x = int((point[0] - origin.x) / resolution)
        y = int((point[1] - origin.y) / resolution)
        return (x, y)
    
    def is_valid_point(self, x: int, y: int) -> bool:
        """Check if a point in map coordinates is valid and obstacle-free"""
        if self.obstacle_map is None:
            return True
            
        width = self.obstacle_map.info.width
        height = self.obstacle_map.info.height
        
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
            
        index = y * width + x
        return self.obstacle_map.data[index] < 50  # Assuming 0-100 occupancy values
    
    def waypoints_to_path(self, waypoints: PoseArray) -> Path:
        """Convert PoseArray to Path message"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = waypoints.poses
        return path
    
    def points_to_path(self, points: np.ndarray) -> Path:
        """Convert numpy array of points to Path message"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path.poses.append(pose)
        
        return path
    
    def publish_path_visualization(self, path: Path):
        """Publish path visualization markers"""
        marker_array = MarkerArray()
        
        # Path line strip
        line_strip = Marker()
        line_strip.header = path.header
        line_strip.ns = "path"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1  # Line width
        line_strip.color.a = 1.0
        line_strip.color.r = 0.0
        line_strip.color.g = 1.0
        line_strip.color.b = 0.0
        
        # Add path points
        for pose in path.poses:
            line_strip.points.append(pose.pose.position)
        
        marker_array.markers.append(line_strip)
        
        # Waypoint spheres
        for i, pose in enumerate(path.poses):
            sphere = Marker()
            sphere.header = path.header
            sphere.ns = "waypoints"
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.a = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            
            marker_array.markers.append(sphere)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 