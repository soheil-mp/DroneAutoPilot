#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from std_msgs.msg import Float32
import numpy as np
import time
import pytest

from src.utilities.mission_controller import MissionState
from tests.fixtures.test_data import (
    create_test_poses,
    create_test_path,
    create_test_mission_params
)

@pytest.mark.integration
class TestFullSystem(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Set up test fixtures"""
        rclpy.init()
        cls.node = Node('test_node')
        cls.setup_test_data()
        cls.setup_subscribers()
        cls.setup_publishers()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test fixtures"""
        cls.node.destroy_node()
        rclpy.shutdown()
        
    @classmethod
    def setup_test_data(cls):
        """Set up test data"""
        cls.test_poses = create_test_poses()
        cls.test_path = create_test_path()
        cls.test_mission_params = create_test_mission_params()
        cls.received_messages = {}
        
    @classmethod
    def setup_subscribers(cls):
        """Set up ROS subscribers"""
        cls.subscribers = {
            'local_pos': cls.node.create_subscription(
                PoseStamped,
                '/mavros/local_position/pose',
                lambda msg: cls.message_callback('local_pos', msg),
                10
            ),
            'current_state': cls.node.create_subscription(
                State,
                '/mavros/state',
                lambda msg: cls.message_callback('current_state', msg),
                10
            ),
            'mission_path': cls.node.create_subscription(
                Path,
                '/skypilot/mission/path',
                lambda msg: cls.message_callback('mission_path', msg),
                10
            ),
            'mission_progress': cls.node.create_subscription(
                Float32,
                '/skypilot/mission/progress',
                lambda msg: cls.message_callback('mission_progress', msg),
                10
            )
        }
        
    @classmethod
    def setup_publishers(cls):
        """Set up ROS publishers"""
        cls.publishers = {
            'goal_pose': cls.node.create_publisher(
                PoseStamped,
                '/skypilot/mission/goal',
                10
            ),
            'mission_state': cls.node.create_publisher(
                State,
                '/skypilot/mission/state',
                10
            )
        }
        
    @classmethod
    def message_callback(cls, topic, msg):
        """Store received messages"""
        cls.received_messages[topic] = msg
        
    def wait_for_messages(self, topics, timeout=10.0):
        """Wait for messages on specified topics"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(topic in self.received_messages for topic in topics):
                return True
        return False
        
    def test_mission_execution(self):
        """Test full mission execution"""
        # Wait for MAVROS connection
        self.assertTrue(
            self.wait_for_messages(['current_state']),
            "Failed to receive MAVROS state"
        )
        
        # Check initial state
        self.assertEqual(
            self.received_messages['current_state'].mode,
            "OFFBOARD"
        )
        self.assertTrue(self.received_messages['current_state'].armed)
        
        # Send waypoint mission
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 5.0
        goal_pose.pose.position.y = 5.0
        goal_pose.pose.position.z = 3.0
        
        self.publishers['goal_pose'].publish(goal_pose)
        
        # Wait for path generation
        self.assertTrue(
            self.wait_for_messages(['mission_path']),
            "Failed to receive mission path"
        )
        
        # Check path validity
        path = self.received_messages['mission_path']
        self.assertGreater(len(path.poses), 0)
        
        # Wait for drone to start moving
        self.assertTrue(
            self.wait_for_messages(['local_pos']),
            "Failed to receive local position"
        )
        
        # Monitor mission progress
        start_time = time.time()
        max_mission_time = 60.0  # 1 minute timeout
        
        while time.time() - start_time < max_mission_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if 'mission_progress' in self.received_messages:
                progress = self.received_messages['mission_progress'].data
                if progress >= 1.0:
                    break
                
            # Check if drone is making progress
            if 'local_pos' in self.received_messages:
                current_pos = self.received_messages['local_pos'].pose.position
                distance_to_goal = np.sqrt(
                    (current_pos.x - goal_pose.pose.position.x)**2 +
                    (current_pos.y - goal_pose.pose.position.y)**2 +
                    (current_pos.z - goal_pose.pose.position.z)**2
                )
                if distance_to_goal < 0.2:  # Goal reached threshold
                    break
                    
        # Check final position
        self.assertTrue(
            self.wait_for_messages(['local_pos']),
            "Failed to receive final position"
        )
        
        final_pos = self.received_messages['local_pos'].pose.position
        distance_to_goal = np.sqrt(
            (final_pos.x - goal_pose.pose.position.x)**2 +
            (final_pos.y - goal_pose.pose.position.y)**2 +
            (final_pos.z - goal_pose.pose.position.z)**2
        )
        
        self.assertLess(distance_to_goal, 0.2, "Failed to reach goal position")
        
    def test_obstacle_avoidance(self):
        """Test obstacle avoidance during mission"""
        # TODO: Implement obstacle avoidance test
        pass
        
    def test_mission_abort(self):
        """Test mission abort handling"""
        # TODO: Implement mission abort test
        pass
        
    def test_battery_failsafe(self):
        """Test battery failsafe behavior"""
        # TODO: Implement battery failsafe test
        pass
        
    def test_geofence_enforcement(self):
        """Test geofence enforcement"""
        # TODO: Implement geofence test
        pass

if __name__ == '__main__':
    unittest.main() 