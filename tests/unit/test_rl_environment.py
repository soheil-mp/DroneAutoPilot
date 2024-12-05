#!/usr/bin/env python3

import unittest
import numpy as np
import gymnasium as gym
from src.utilities.rl_environment import DroneNavigationEnv

class TestDroneNavigationEnv(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.env = DroneNavigationEnv()
        self.setup_test_data()
        
    def setup_test_data(self):
        """Create test data"""
        # Test observation space bounds
        self.test_obs = {
            'position': np.array([1.0, 2.0, 3.0]),
            'velocity': np.array([0.5, 0.5, 0.0]),
            'goal': np.array([5.0, 5.0, 3.0]),
            'obstacles': np.zeros((10, 10))  # Empty obstacle map
        }
        
        # Test action
        self.test_action = np.array([0.5, 0.5, 0.0, 0.1])  # vx, vy, vz, yaw_rate
        
    def test_reset(self):
        """Test environment reset"""
        obs, info = self.env.reset()
        
        # Check observation space
        self.assertIn('position', obs)
        self.assertIn('velocity', obs)
        self.assertIn('goal', obs)
        self.assertIn('obstacles', obs)
        
        # Check observation shapes
        self.assertEqual(obs['position'].shape, (3,))
        self.assertEqual(obs['velocity'].shape, (3,))
        self.assertEqual(obs['goal'].shape, (3,))
        self.assertTrue(len(obs['obstacles'].shape) == 2)  # 2D obstacle map
        
        # Check initial position bounds
        self.assertTrue(np.all(obs['position'] >= self.env.observation_space['position'].low))
        self.assertTrue(np.all(obs['position'] <= self.env.observation_space['position'].high))
        
    def test_step(self):
        """Test environment step"""
        self.env.reset()
        obs, reward, terminated, truncated, info = self.env.step(self.test_action)
        
        # Check observation validity
        self.assertTrue(np.all(obs['position'] >= self.env.observation_space['position'].low))
        self.assertTrue(np.all(obs['position'] <= self.env.observation_space['position'].high))
        self.assertTrue(np.all(obs['velocity'] >= self.env.observation_space['velocity'].low))
        self.assertTrue(np.all(obs['velocity'] <= self.env.observation_space['velocity'].high))
        
        # Check reward bounds
        self.assertGreaterEqual(reward, self.env.reward_range[0])
        self.assertLessEqual(reward, self.env.reward_range[1])
        
        # Check termination flags
        self.assertIsInstance(terminated, bool)
        self.assertIsInstance(truncated, bool)
        
    def test_reward_calculation(self):
        """Test reward calculation"""
        # Test goal reached
        self.env.state = {
            'position': np.array([5.0, 5.0, 3.0]),
            'velocity': np.array([0.0, 0.0, 0.0])
        }
        self.env.goal = np.array([5.0, 5.0, 3.0])
        
        reward = self.env.calculate_reward()
        self.assertEqual(reward, self.env.config['rewards']['goal_reached'])
        
        # Test collision
        self.env.state['position'] = np.array([0.0, 0.0, 0.0])
        obstacle_map = np.zeros((10, 10))
        obstacle_map[0, 0] = 1
        self.env.obstacle_map = obstacle_map
        
        reward = self.env.calculate_reward()
        self.assertEqual(reward, self.env.config['rewards']['collision'])
        
        # Test progress reward
        self.env.state['position'] = np.array([2.5, 2.5, 3.0])
        self.env.previous_distance = 5.0
        self.env.obstacle_map = np.zeros((10, 10))
        
        reward = self.env.calculate_reward()
        self.assertGreater(reward, 0)  # Should get positive reward for moving closer
        
    def test_observation_space(self):
        """Test observation space configuration"""
        obs_space = self.env.observation_space
        
        # Check space components
        self.assertIn('position', obs_space.spaces)
        self.assertIn('velocity', obs_space.spaces)
        self.assertIn('goal', obs_space.spaces)
        self.assertIn('obstacles', obs_space.spaces)
        
        # Check position bounds
        pos_space = obs_space['position']
        self.assertEqual(pos_space.shape, (3,))
        self.assertTrue(np.all(pos_space.low == self.env.config['observation']['position_min']))
        self.assertTrue(np.all(pos_space.high == self.env.config['observation']['position_max']))
        
    def test_action_space(self):
        """Test action space configuration"""
        action_space = self.env.action_space
        
        # Check action space type and shape
        self.assertEqual(action_space.shape, (4,))  # vx, vy, vz, yaw_rate
        
        # Check action bounds
        self.assertTrue(np.all(action_space.low == self.env.config['action']['min_action']))
        self.assertTrue(np.all(action_space.high == self.env.config['action']['max_action']))
        
    def test_goal_reached(self):
        """Test goal reached detection"""
        # Test goal not reached
        self.env.state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([0.0, 0.0, 0.0])
        }
        self.env.goal = np.array([5.0, 5.0, 3.0])
        
        self.assertFalse(self.env.is_goal_reached())
        
        # Test goal reached
        self.env.state['position'] = np.array([5.0, 5.0, 3.0])
        self.assertTrue(self.env.is_goal_reached())
        
        # Test goal reached with tolerance
        self.env.state['position'] = np.array([4.9, 5.1, 3.0])
        self.assertTrue(self.env.is_goal_reached())
        
    def test_collision_detection(self):
        """Test collision detection"""
        # Test no collision
        self.env.state = {
            'position': np.array([5.0, 5.0, 3.0])
        }
        self.env.obstacle_map = np.zeros((10, 10))
        
        self.assertFalse(self.env.is_collision())
        
        # Test collision
        self.env.state['position'] = np.array([0.0, 0.0, 0.0])
        obstacle_map = np.zeros((10, 10))
        obstacle_map[0, 0] = 1
        self.env.obstacle_map = obstacle_map
        
        self.assertTrue(self.env.is_collision())
        
    def test_state_normalization(self):
        """Test state normalization"""
        # Test position normalization
        raw_pos = np.array([5.0, 5.0, 3.0])
        norm_pos = self.env.normalize_position(raw_pos)
        
        self.assertTrue(np.all(norm_pos >= -1.0))
        self.assertTrue(np.all(norm_pos <= 1.0))
        
        # Test velocity normalization
        raw_vel = np.array([2.0, -1.0, 0.5])
        norm_vel = self.env.normalize_velocity(raw_vel)
        
        self.assertTrue(np.all(norm_vel >= -1.0))
        self.assertTrue(np.all(norm_vel <= 1.0))
        
    def test_episode_termination(self):
        """Test episode termination conditions"""
        self.env.reset()
        
        # Test max steps termination
        for _ in range(self.env.max_steps + 1):
            _, _, terminated, truncated, _ = self.env.step(self.test_action)
            if terminated or truncated:
                break
        
        self.assertTrue(terminated or truncated)
        
        # Test goal reached termination
        self.env.reset()
        self.env.state['position'] = self.env.goal
        _, _, terminated, _, _ = self.env.step(self.test_action)
        
        self.assertTrue(terminated)
        
        # Test collision termination
        self.env.reset()
        self.env.state['position'] = np.array([0.0, 0.0, 0.0])
        obstacle_map = np.zeros((10, 10))
        obstacle_map[0, 0] = 1
        self.env.obstacle_map = obstacle_map
        
        _, _, terminated, _, _ = self.env.step(self.test_action)
        self.assertTrue(terminated)

if __name__ == '__main__':
    unittest.main() 