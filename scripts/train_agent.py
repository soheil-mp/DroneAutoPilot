#!/usr/bin/env python3

import os
import yaml
import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from drone_nav_rl.src.environments.drone_env import DroneNavEnv
from drone_nav_rl.src.networks.policy_network import DroneActorCriticPolicy
from drone_nav_rl.src.networks.network_utils import create_policy

def load_config():
    """Load configuration from yaml file"""
    config_path = os.path.join(
        os.path.dirname(__file__),
        '../config/rl_params.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['rl_training']

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Load configuration
    config = load_config()
    
    # Create and wrap the environment
    env = DummyVecEnv([lambda: DroneNavEnv()])
    
    # Create directories for logs and models
    log_dir = "logs"
    model_dir = "models"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)
    
    # Create callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=config['training']['save_freq'],
        save_path=model_dir,
        name_prefix="drone_nav_model"
    )
    
    eval_env = DummyVecEnv([lambda: DroneNavEnv()])
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=f"{model_dir}/best_model",
        log_path=log_dir,
        eval_freq=config['training']['eval_freq'],
        n_eval_episodes=config['training']['n_eval_episodes'],
        deterministic=True
    )
    
    # Policy kwargs for custom network architecture
    policy_kwargs = {
        "features_extractor_class": DroneActorCriticPolicy,
        "features_extractor_kwargs": dict(features_dim=256),
        "net_arch": [dict(
            pi=config['network']['net_arch']['pi'],
            vf=config['network']['net_arch']['vf']
        )]
    }
    
    # Initialize the agent with custom policy
    model = PPO(
        policy=DroneActorCriticPolicy,
        env=env,
        learning_rate=config['hyperparameters']['learning_rate'],
        n_steps=config['hyperparameters']['n_steps'],
        batch_size=config['hyperparameters']['batch_size'],
        n_epochs=config['hyperparameters']['n_epochs'],
        gamma=config['hyperparameters']['gamma'],
        gae_lambda=config['hyperparameters']['gae_lambda'],
        clip_range=config['hyperparameters']['clip_range'],
        ent_coef=config['hyperparameters']['ent_coef'],
        vf_coef=config['hyperparameters']['vf_coef'],
        max_grad_norm=config['hyperparameters']['max_grad_norm'],
        tensorboard_log=log_dir,
        policy_kwargs=policy_kwargs,
        verbose=1
    )
    
    # Train the agent
    try:
        model.learn(
            total_timesteps=config['training']['total_timesteps'],
            callback=[checkpoint_callback, eval_callback],
            log_interval=config['training']['log_interval']
        )
        
        # Save the final model
        model.save(f"{model_dir}/final_model")
        
    except KeyboardInterrupt:
        print("\nTraining interrupted! Saving model...")
        model.save(f"{model_dir}/interrupted_model")
    
    # Cleanup
    env.close()
    eval_env.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 