#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header
import numpy as np

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Create publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        # Create subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            10
        )
        
        # Create service clients
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        
        # Initialize state variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Create timer for the control loop
        self.create_timer(0.02, self.control_loop)  # 50Hz
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def position_callback(self, msg):
        self.current_pose = msg
        
    async def arm(self):
        """Arm the drone"""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
            
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        
        future = self.arming_client.call_async(arm_cmd)
        await future
        
        return future.result().success
        
    async def set_mode(self, mode):
        """Set the flight mode"""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
            
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = mode
        
        future = self.set_mode_client.call_async(mode_cmd)
        await future
        
        return future.result().mode_sent
        
    def send_pose(self, x, y, z, yaw=0.0):
        """Send position setpoint"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Convert yaw to quaternion (simplified)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(yaw / 2.0)
        pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        self.local_pos_pub.publish(pose)
        
    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity setpoint"""
        vel = TwistStamped()
        vel.header = Header()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = "map"
        
        vel.twist.linear.x = vx
        vel.twist.linear.y = vy
        vel.twist.linear.z = vz
        vel.twist.angular.z = yaw_rate
        
        self.vel_pub.publish(vel)
        
    def control_loop(self):
        """Main control loop"""
        if not self.current_state.armed:
            # Try to arm
            self.arm()
            return
            
        if self.current_state.mode != "OFFBOARD":
            # Try to set mode
            self.set_mode("OFFBOARD")
            return
            
        # Send setpoints (example hover at 2m)
        self.send_pose(0.0, 0.0, 2.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 