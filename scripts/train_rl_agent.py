#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from skypilot.drone_env import DroneNavigationEnv
from skypilot.policy_network import PolicyNetwork
import torch
import yaml
import os

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Load training parameters
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(pkg_dir, 'config', 'rl_params.yaml'), 'r') as f:
        params = yaml.safe_load(f)
    
    # Create environment
    env = DroneNavigationEnv()
    
    # Create policy network
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    policy = PolicyNetwork(state_dim, action_dim)
    
    # Training loop
    try:
        for episode in range(params['num_episodes']):
            state = env.reset()
            episode_reward = 0
            
            for step in range(params['max_steps']):
                # Get action from policy
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                action = policy.select_action(state_tensor)
                
                # Take action in environment
                next_state, reward, done, _ = env.step(action)
                
                # Store transition and train
                policy.update([state, action, reward, next_state, done])
                
                state = next_state
                episode_reward += reward
                
                if done:
                    break
            
            print(f"Episode {episode}, Reward: {episode_reward}")
            
            # Save model periodically
            if episode % params['save_interval'] == 0:
                torch.save(policy.state_dict(), 
                         os.path.join(pkg_dir, 'models', f'policy_{episode}.pth'))
    
    except KeyboardInterrupt:
        print("Training interrupted")
    
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
