#!/usr/bin/env python3

import os
import yaml
import numpy as np
from typing import List, Dict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Vector3
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from drone_nav_rl_interfaces.msg import MissionStatus

class MissionVisualization(Node):
    def __init__(self):
        super().__init__('mission_visualization')
        
        # Load configuration
        self.config = self.load_config()
        
        # Initialize state
        self.current_mission_status = None
        self.current_path = None
        self.drone_pose = None
        self.obstacle_map = None
        
        # Create publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization/markers',
            10
        )
        
        # Create subscribers
        self.mission_status_sub = self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.mission_status_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.path_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Create timer for visualization updates
        self.create_timer(0.1, self.update_visualization)  # 10Hz
        
        # Initialize marker IDs
        self.next_marker_id = 0
        
    def load_config(self) -> Dict:
        """Load configuration from yaml file"""
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../config/mission_params.yaml'
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)['mission_control']
    
    def mission_status_callback(self, msg: MissionStatus):
        """Handle mission status updates"""
        self.current_mission_status = msg
        
    def path_callback(self, msg: Path):
        """Handle path updates"""
        self.current_path = msg
        
    def pose_callback(self, msg: PoseStamped):
        """Handle drone pose updates"""
        self.drone_pose = msg
        
    def map_callback(self, msg: OccupancyGrid):
        """Handle map updates"""
        self.obstacle_map = msg
        
    def update_visualization(self):
        """Update all visualization markers"""
        marker_array = MarkerArray()
        
        # Add drone marker
        if self.drone_pose is not None:
            marker_array.markers.append(self.create_drone_marker())
        
        # Add path markers
        if self.current_path is not None:
            marker_array.markers.extend(self.create_path_markers())
        
        # Add mission status markers
        if self.current_mission_status is not None:
            marker_array.markers.extend(self.create_status_markers())
        
        # Add geofence markers
        marker_array.markers.extend(self.create_geofence_markers())
        
        # Add obstacle markers
        if self.obstacle_map is not None:
            marker_array.markers.extend(self.create_obstacle_markers())
        
        # Publish all markers
        self.marker_pub.publish(marker_array)
        
    def create_drone_marker(self) -> Marker:
        """Create marker for drone visualization"""
        marker = Marker()
        marker.header = self.drone_pose.header
        marker.ns = "drone"
        marker.id = self.get_next_marker_id()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://drone_nav_rl/meshes/quadrotor.dae"
        marker.action = Marker.ADD
        marker.pose = self.drone_pose.pose
        marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        return marker
        
    def create_path_markers(self) -> List[Marker]:
        """Create markers for path visualization"""
        markers = []
        
        # Path line strip
        line_strip = Marker()
        line_strip.header = self.current_path.header
        line_strip.ns = "path"
        line_strip.id = self.get_next_marker_id()
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1
        line_strip.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        line_strip.points = [pose.pose.position for pose in self.current_path.poses]
        markers.append(line_strip)
        
        # Waypoint spheres
        for i, pose in enumerate(self.current_path.poses):
            sphere = Marker()
            sphere.header = self.current_path.header
            sphere.ns = "waypoints"
            sphere.id = self.get_next_marker_id()
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale = Vector3(x=0.3, y=0.3, z=0.3)
            
            # Color based on waypoint status
            if self.current_mission_status is not None:
                if i < self.current_mission_status.current_waypoint:
                    sphere.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray for completed
                elif i == self.current_mission_status.current_waypoint:
                    sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow for current
                else:
                    sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red for upcoming
            else:
                sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                
            markers.append(sphere)
        
        return markers
        
    def create_status_markers(self) -> List[Marker]:
        """Create markers for mission status visualization"""
        markers = []
        
        # Mission progress text
        text = Marker()
        text.header.frame_id = "map"
        text.ns = "status"
        text.id = self.get_next_marker_id()
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 5.0  # Display above mission area
        text.scale.z = 0.5  # Text size
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Format status text
        status_text = (
            f"Mission: {self.current_mission_status.mission_id}\n"
            f"State: {self.current_mission_status.state}\n"
            f"Progress: {self.current_mission_status.mission_progress * 100:.1f}%\n"
            f"Battery: {self.current_mission_status.battery_remaining:.1f}%"
        )
        text.text = status_text
        markers.append(text)
        
        return markers
        
    def create_geofence_markers(self) -> List[Marker]:
        """Create markers for geofence visualization"""
        markers = []
        
        if not self.config['geofence']['enabled']:
            return markers
            
        # Geofence boundary lines
        boundary = Marker()
        boundary.header.frame_id = "map"
        boundary.ns = "geofence"
        boundary.id = self.get_next_marker_id()
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.1
        boundary.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5)
        
        # Add boundary points
        points = self.config['geofence']['boundary_points']
        for point in points + [points[0]]:  # Close the loop
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = self.config['geofence']['max_altitude']
            boundary.points.append(p)
            
        markers.append(boundary)
        
        # Add altitude limits
        for point in points:
            line = Marker()
            line.header.frame_id = "map"
            line.ns = "geofence_altitude"
            line.id = self.get_next_marker_id()
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.3)
            
            # Add vertical line from min to max altitude
            p1 = Point(x=point[0], y=point[1], z=self.config['geofence']['min_altitude'])
            p2 = Point(x=point[0], y=point[1], z=self.config['geofence']['max_altitude'])
            line.points = [p1, p2]
            
            markers.append(line)
        
        return markers
        
    def create_obstacle_markers(self) -> List[Marker]:
        """Create markers for obstacle visualization"""
        markers = []
        
        if self.obstacle_map is None:
            return markers
            
        # Create grid cells for obstacles
        cells = Marker()
        cells.header = self.obstacle_map.header
        cells.ns = "obstacles"
        cells.id = self.get_next_marker_id()
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.scale.x = self.obstacle_map.info.resolution
        cells.scale.y = self.obstacle_map.info.resolution
        cells.scale.z = 0.1
        
        for y in range(self.obstacle_map.info.height):
            for x in range(self.obstacle_map.info.width):
                idx = y * self.obstacle_map.info.width + x
                if self.obstacle_map.data[idx] > 50:  # Occupied cell
                    p = Point()
                    p.x = self.obstacle_map.info.origin.position.x + (x + 0.5) * self.obstacle_map.info.resolution
                    p.y = self.obstacle_map.info.origin.position.y + (y + 0.5) * self.obstacle_map.info.resolution
                    p.z = 0.0
                    cells.points.append(p)
                    
                    # Color based on occupancy value
                    intensity = min(self.obstacle_map.data[idx] / 100.0, 1.0)
                    color = ColorRGBA(r=intensity, g=0.0, b=0.0, a=0.8)
                    cells.colors.append(color)
        
        markers.append(cells)
        return markers
        
    def get_next_marker_id(self) -> int:
        """Get next available marker ID"""
        marker_id = self.next_marker_id
        self.next_marker_id += 1
        return marker_id

def main(args=None):
    rclpy.init(args=args)
    mission_viz = MissionVisualization()
    
    try:
        rclpy.spin(mission_viz)
    except KeyboardInterrupt:
        pass
    finally:
        mission_viz.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 