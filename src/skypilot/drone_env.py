import gym
import numpy as np
from gym import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class DroneNavEnv(gym.Env):
    """Custom Environment for drone navigation that follows gym interface"""
    
    def __init__(self):
        super(DroneNavEnv, self).__init__()
        
        # Initialize ROS2 node
        rclpy.init()
        self.node = Node('drone_nav_env')
        
        # Define action and observation space
        # Action space: [vx, vy, vz, yaw_rate]
        self.action_space = spaces.Box(
            low=np.array([-1, -1, -1, -1]),
            high=np.array([1, 1, 1, 1]),
            dtype=np.float32
        )
        
        # Observation space: [position(3), velocity(3), orientation(3), lidar_scan(16)]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(25,),
            dtype=np.float32
        )
        
        # ROS2 publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(
            TwistStamped,
            '/drone/cmd_vel',
            10
        )
        
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            '/drone/pose',
            self._pose_callback,
            10
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/drone/odom',
            self._odom_callback,
            10
        )
        
        self.lidar_sub = self.node.create_subscription(
            LaserScan,
            '/drone/scan',
            self._lidar_callback,
            10
        )
        
        # Initialize state variables
        self.current_pose = None
        self.current_vel = None
        self.current_scan = None
        self.goal_position = None
        
    def _pose_callback(self, msg):
        self.current_pose = msg
        
    def _odom_callback(self, msg):
        self.current_vel = msg
        
    def _lidar_callback(self, msg):
        self.current_scan = msg
        
    def reset(self):
        """Reset the environment to initial state"""
        # Reset drone position
        self.goal_position = np.random.uniform(-5, 5, 3)  # Random goal
        
        # Wait for valid sensor data
        while (self.current_pose is None or 
               self.current_vel is None or 
               self.current_scan is None):
            rclpy.spin_once(self.node)
            
        return self._get_observation()
        
    def step(self, action):
        """Execute one time step within the environment"""
        # Apply action
        cmd_vel = TwistStamped()
        cmd_vel.twist.linear.x = action[0]
        cmd_vel.twist.linear.y = action[1]
        cmd_vel.twist.linear.z = action[2]
        cmd_vel.twist.angular.z = action[3]
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Update environment (ROS2 spin)
        rclpy.spin_once(self.node)
        
        # Get new observation
        obs = self._get_observation()
        
        # Calculate reward
        reward = self._compute_reward()
        
        # Check if episode is done
        done = self._is_done()
        
        return obs, reward, done, {}
        
    def _get_observation(self):
        """Convert ROS messages to observation vector"""
        if (self.current_pose is None or 
            self.current_vel is None or 
            self.current_scan is None):
            return np.zeros(self.observation_space.shape[0])
            
        pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        vel = np.array([
            self.current_vel.twist.twist.linear.x,
            self.current_vel.twist.twist.linear.y,
            self.current_vel.twist.twist.linear.z
        ])
        
        # Convert quaternion to euler angles
        orientation = np.array([0, 0, 0])  # Simplified for now
        
        # Process lidar data (downsample to 16 points)
        scan = np.array(self.current_scan.ranges)
        scan = np.clip(scan, 0, self.current_scan.range_max)
        scan = np.array_split(scan, 16)
        scan = np.array([np.min(s) for s in scan])
        
        return np.concatenate([pos, vel, orientation, scan])
        
    def _compute_reward(self):
        """Calculate reward based on current state"""
        if self.current_pose is None or self.goal_position is None:
            return 0.0
            
        # Distance to goal
        pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        distance = np.linalg.norm(pos - self.goal_position)
        
        # Collision penalty
        min_distance = min(self.current_scan.ranges)
        collision_penalty = -10.0 if min_distance < 0.5 else 0.0
        
        # Goal reward
        goal_reward = 10.0 if distance < 0.5 else 0.0
        
        # Combine rewards
        reward = -distance + collision_penalty + goal_reward
        
        return reward
        
    def _is_done(self):
        """Check if episode should end"""
        if self.current_pose is None or self.goal_position is None:
            return False
            
        # Check if goal is reached
        pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        distance = np.linalg.norm(pos - self.goal_position)
        
        # Check for collision
        min_distance = min(self.current_scan.ranges)
        
        return distance < 0.5 or min_distance < 0.3
        
    def close(self):
        """Clean up resources"""
        self.node.destroy_node()
        rclpy.shutdown() 