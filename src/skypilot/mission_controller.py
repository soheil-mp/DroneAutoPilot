#!/usr/bin/env python3

import os
import yaml
import numpy as np
from enum import Enum
from typing import List, Dict, Optional
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from drone_nav_rl_interfaces.action import ExecuteMission
from drone_nav_rl_interfaces.msg import MissionStatus, WaypointList
from drone_nav_rl_interfaces.srv import LoadMission, AbortMission

class MissionState(Enum):
    IDLE = "IDLE"
    LOADING = "LOADING"
    EXECUTING = "EXECUTING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Load mission parameters
        self.config = self.load_config()
        
        # Initialize state
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.current_waypoint_index = 0
        self.mission_waypoints = []
        self.home_position = None
        
        # Create publishers
        self.status_pub = self.create_publisher(
            MissionStatus,
            '/mission/status',
            10
        )
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/mission/target_pose',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/mission/path',
            10
        )
        
        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Create services
        self.load_mission_srv = self.create_service(
            LoadMission,
            '/mission/load',
            self.load_mission_callback
        )
        
        self.abort_mission_srv = self.create_service(
            AbortMission,
            '/mission/abort',
            self.abort_mission_callback
        )
        
        # Create action server
        self._action_server = ActionServer(
            self,
            ExecuteMission,
            '/mission/execute',
            self.execute_mission_callback
        )
        
        # Create timer for status updates
        self.create_timer(1.0, self.publish_status)
        
    def load_config(self) -> Dict:
        """Load mission configuration from yaml file"""
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../config/mission_params.yaml'
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)['mission_control']
    
    def pose_callback(self, msg: PoseStamped):
        """Store current drone position"""
        if self.home_position is None:
            self.home_position = msg.pose.position
    
    def load_mission_callback(self, request, response):
        """Handle mission loading service requests"""
        if self.current_state != MissionState.IDLE:
            response.success = False
            response.message = "Cannot load mission while another mission is active"
            return response
        
        try:
            self.current_state = MissionState.LOADING
            self.current_mission = request.mission_type
            self.mission_waypoints = self.generate_waypoints(
                request.mission_type,
                request.parameters
            )
            
            # Publish the planned path
            self.publish_path()
            
            self.current_state = MissionState.IDLE
            response.success = True
            response.message = "Mission loaded successfully"
            
        except Exception as e:
            self.current_state = MissionState.IDLE
            response.success = False
            response.message = f"Failed to load mission: {str(e)}"
        
        return response
    
    def abort_mission_callback(self, request, response):
        """Handle mission abort service requests"""
        if self.current_state not in [MissionState.EXECUTING, MissionState.PAUSED]:
            response.success = False
            response.message = "No active mission to abort"
            return response
        
        self.current_state = MissionState.ABORTED
        response.success = True
        response.message = "Mission aborted"
        return response
    
    async def execute_mission_callback(self, goal_handle):
        """Handle mission execution action requests"""
        if self.current_state != MissionState.IDLE:
            goal_handle.abort()
            return
        
        self.current_state = MissionState.EXECUTING
        self.current_waypoint_index = 0
        
        feedback_msg = ExecuteMission.Feedback()
        result = ExecuteMission.Result()
        
        try:
            while self.current_waypoint_index < len(self.mission_waypoints):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.current_state = MissionState.ABORTED
                    return
                
                # Send next waypoint
                waypoint = self.mission_waypoints[self.current_waypoint_index]
                self.publish_target_pose(waypoint)
                
                # Update feedback
                feedback_msg.current_waypoint = self.current_waypoint_index
                feedback_msg.total_waypoints = len(self.mission_waypoints)
                goal_handle.publish_feedback(feedback_msg)
                
                # Wait for waypoint completion
                if await self.wait_for_waypoint_completion():
                    self.current_waypoint_index += 1
                else:
                    # Waypoint failed
                    goal_handle.abort()
                    self.current_state = MissionState.ABORTED
                    return
            
            # Mission completed
            self.current_state = MissionState.COMPLETED
            result.success = True
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f"Mission execution failed: {str(e)}")
            goal_handle.abort()
            self.current_state = MissionState.ABORTED
    
    def generate_waypoints(self, mission_type: str, parameters: Dict) -> List[PoseStamped]:
        """Generate waypoints based on mission type and parameters"""
        if mission_type == "waypoint_navigation":
            return self.generate_waypoint_mission(parameters)
        elif mission_type == "area_coverage":
            return self.generate_coverage_mission(parameters)
        elif mission_type == "perimeter_inspection":
            return self.generate_perimeter_mission(parameters)
        elif mission_type == "point_of_interest":
            return self.generate_poi_mission(parameters)
        else:
            raise ValueError(f"Unknown mission type: {mission_type}")
    
    def generate_waypoint_mission(self, parameters: Dict) -> List[PoseStamped]:
        """Generate waypoints for waypoint navigation mission"""
        waypoints = []
        settings = self.config['waypoint_settings']
        
        for point in parameters['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2] if len(point) > 2 else self.config['default_settings']['takeoff_altitude']
            waypoints.append(pose)
        
        return waypoints
    
    def generate_coverage_mission(self, parameters: Dict) -> List[PoseStamped]:
        """Generate waypoints for area coverage mission"""
        settings = self.config['area_coverage']
        boundary = parameters['boundary_points']
        
        # Implement lawn mower pattern
        if settings['pattern'] == "lawn_mower":
            min_x = min(p[0] for p in boundary)
            max_x = max(p[0] for p in boundary)
            min_y = min(p[1] for p in boundary)
            max_y = max(p[1] for p in boundary)
            
            waypoints = []
            y = min_y
            direction = 1
            
            while y <= max_y:
                if direction > 0:
                    x_range = np.arange(min_x, max_x, settings['line_spacing'])
                else:
                    x_range = np.arange(max_x, min_x, -settings['line_spacing'])
                
                for x in x_range:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = self.config['default_settings']['takeoff_altitude']
                    waypoints.append(pose)
                
                y += settings['line_spacing']
                direction *= -1
            
            return waypoints
    
    def generate_perimeter_mission(self, parameters: Dict) -> List[PoseStamped]:
        """Generate waypoints for perimeter inspection mission"""
        settings = self.config['perimeter_inspection']
        boundary = parameters['boundary_points']
        waypoints = []
        
        for i in range(len(boundary)):
            current = boundary[i]
            next_point = boundary[(i + 1) % len(boundary)]
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = current[0]
            pose.pose.position.y = current[1]
            pose.pose.position.z = self.config['default_settings']['takeoff_altitude']
            waypoints.append(pose)
        
        # Close the loop
        waypoints.append(waypoints[0])
        return waypoints
    
    def generate_poi_mission(self, parameters: Dict) -> List[PoseStamped]:
        """Generate waypoints for point of interest mission"""
        settings = self.config['point_of_interest']
        poi = parameters['point']
        waypoints = []
        
        # Generate circular pattern around POI
        num_points = 8  # Number of points in the orbit
        for i in range(num_points + 1):
            angle = 2 * np.pi * i / num_points
            x = poi[0] + settings['orbit_radius'] * np.cos(angle)
            y = poi[1] + settings['orbit_radius'] * np.sin(angle)
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = settings['orbit_height']
            waypoints.append(pose)
        
        return waypoints
    
    def publish_status(self):
        """Publish mission status"""
        msg = MissionStatus()
        msg.state = self.current_state.value
        msg.current_waypoint = self.current_waypoint_index
        msg.total_waypoints = len(self.mission_waypoints)
        self.status_pub.publish(msg)
    
    def publish_path(self):
        """Publish planned mission path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.mission_waypoints
        self.path_pub.publish(path)
    
    def publish_target_pose(self, pose: PoseStamped):
        """Publish target pose for the drone"""
        pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(pose)
    
    async def wait_for_waypoint_completion(self) -> bool:
        """Wait for current waypoint to be reached"""
        # TODO: Implement waypoint completion check
        await self.create_rate(1).sleep()
        return True

def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    
    try:
        rclpy.spin(mission_controller)
    except KeyboardInterrupt:
        pass
    finally:
        mission_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 