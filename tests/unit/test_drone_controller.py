#!/usr/bin/env python3

import unittest
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import State
from src.utilities.drone_controller import DroneController

class TestDroneController(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.drone_controller = DroneController()
        self.setup_test_data()
        
    def setup_test_data(self):
        """Create test data"""
        # Create test poses
        self.test_pose = PoseStamped()
        self.test_pose.pose.position.x = 1.0
        self.test_pose.pose.position.y = 2.0
        self.test_pose.pose.position.z = 3.0
        
        # Create test velocities
        self.test_vel = TwistStamped()
        self.test_vel.twist.linear.x = 0.5
        self.test_vel.twist.linear.y = 0.5
        self.test_vel.twist.linear.z = 0.0
        self.test_vel.twist.angular.z = 0.1
        
        # Create test state
        self.test_state = State()
        self.test_state.armed = True
        self.test_state.mode = "OFFBOARD"
        
    def test_pose_callback(self):
        """Test pose callback handling"""
        self.drone_controller.position_callback(self.test_pose)
        
        self.assertEqual(
            self.drone_controller.current_pose.pose.position.x,
            self.test_pose.pose.position.x
        )
        self.assertEqual(
            self.drone_controller.current_pose.pose.position.y,
            self.test_pose.pose.position.y
        )
        self.assertEqual(
            self.drone_controller.current_pose.pose.position.z,
            self.test_pose.pose.position.z
        )
        
    def test_state_callback(self):
        """Test state callback handling"""
        self.drone_controller.state_callback(self.test_state)
        
        self.assertEqual(
            self.drone_controller.current_state.armed,
            self.test_state.armed
        )
        self.assertEqual(
            self.drone_controller.current_state.mode,
            self.test_state.mode
        )
        
    def test_send_pose(self):
        """Test sending position setpoint"""
        x, y, z = 1.0, 2.0, 3.0
        yaw = 0.5
        
        self.drone_controller.send_pose(x, y, z, yaw)
        
        # Get the last published message
        last_msg = self.drone_controller.local_pos_pub.get_last_message()
        
        self.assertEqual(last_msg.pose.position.x, x)
        self.assertEqual(last_msg.pose.position.y, y)
        self.assertEqual(last_msg.pose.position.z, z)
        
        # Check yaw conversion to quaternion
        quat = last_msg.pose.orientation
        yaw_back = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                             1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        self.assertAlmostEqual(yaw_back, yaw, places=4)
        
    def test_send_velocity(self):
        """Test sending velocity setpoint"""
        vx, vy, vz = 0.5, 0.5, 0.0
        yaw_rate = 0.1
        
        self.drone_controller.send_velocity(vx, vy, vz, yaw_rate)
        
        # Get the last published message
        last_msg = self.drone_controller.vel_pub.get_last_message()
        
        self.assertEqual(last_msg.twist.linear.x, vx)
        self.assertEqual(last_msg.twist.linear.y, vy)
        self.assertEqual(last_msg.twist.linear.z, vz)
        self.assertEqual(last_msg.twist.angular.z, yaw_rate)
        
    def test_arming(self):
        """Test arming command"""
        # Test arming request
        success = self.drone_controller.arm()
        self.assertTrue(success)
        
        # Test arming with already armed state
        self.drone_controller.current_state.armed = True
        success = self.drone_controller.arm()
        self.assertTrue(success)
        
    def test_set_mode(self):
        """Test mode setting"""
        # Test setting OFFBOARD mode
        success = self.drone_controller.set_mode("OFFBOARD")
        self.assertTrue(success)
        
        # Test setting invalid mode
        success = self.drone_controller.set_mode("INVALID_MODE")
        self.assertFalse(success)
        
    def test_control_loop(self):
        """Test main control loop"""
        # Test control loop with disarmed state
        self.drone_controller.current_state.armed = False
        self.drone_controller.control_loop()
        
        # Should attempt to arm
        self.assertTrue(self.drone_controller.arming_client.called)
        
        # Test control loop with armed but not OFFBOARD
        self.drone_controller.current_state.armed = True
        self.drone_controller.current_state.mode = "MANUAL"
        self.drone_controller.control_loop()
        
        # Should attempt to set OFFBOARD mode
        self.assertTrue(self.drone_controller.set_mode_client.called)
        
        # Test control loop with armed and OFFBOARD
        self.drone_controller.current_state.mode = "OFFBOARD"
        self.drone_controller.control_loop()
        
        # Should send setpoint
        self.assertTrue(self.drone_controller.local_pos_pub.called)
        
    def test_quaternion_conversion(self):
        """Test quaternion conversion utilities"""
        # Test euler to quaternion
        roll, pitch, yaw = 0.1, 0.2, 0.3
        quat = self.drone_controller.euler_to_quaternion(roll, pitch, yaw)
        
        # Convert back to euler
        roll_back, pitch_back, yaw_back = self.drone_controller.quaternion_to_euler(quat)
        
        # Check conversion accuracy
        self.assertAlmostEqual(roll, roll_back, places=4)
        self.assertAlmostEqual(pitch, pitch_back, places=4)
        self.assertAlmostEqual(yaw, yaw_back, places=4)
        
    def test_waypoint_navigation(self):
        """Test waypoint navigation"""
        waypoint = PoseStamped()
        waypoint.pose.position.x = 5.0
        waypoint.pose.position.y = 5.0
        waypoint.pose.position.z = 3.0
        
        # Set current position
        self.drone_controller.current_pose = self.test_pose
        
        # Test navigation update
        cmd_vel = self.drone_controller.navigate_to_waypoint(waypoint)
        
        # Velocity should be in direction of waypoint
        self.assertGreater(cmd_vel.twist.linear.x, 0)
        self.assertGreater(cmd_vel.twist.linear.y, 0)
        
        # Test waypoint reached
        self.drone_controller.current_pose.pose.position.x = 5.0
        self.drone_controller.current_pose.pose.position.y = 5.0
        self.drone_controller.current_pose.pose.position.z = 3.0
        
        cmd_vel = self.drone_controller.navigate_to_waypoint(waypoint)
        
        # Velocity should be zero
        self.assertAlmostEqual(cmd_vel.twist.linear.x, 0.0)
        self.assertAlmostEqual(cmd_vel.twist.linear.y, 0.0)
        self.assertAlmostEqual(cmd_vel.twist.linear.z, 0.0)

if __name__ == '__main__':
    unittest.main() 