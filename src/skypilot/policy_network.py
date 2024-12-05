import torch
import torch.nn as nn
import numpy as np
from typing import Dict, List, Tuple, Type, Union
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class DroneFeatureExtractor(BaseFeaturesExtractor):
    """
    Custom feature extractor for drone state observations.
    Processes position, velocity, orientation, and lidar data separately.
    """
    
    def __init__(self, observation_space, features_dim: int = 256):
        super().__init__(observation_space, features_dim)
        
        # Input dimensions
        self.pos_dim = 3  # x, y, z
        self.vel_dim = 3  # vx, vy, vz
        self.orient_dim = 3  # roll, pitch, yaw
        self.lidar_dim = 16  # 16 lidar points
        
        # Feature extraction layers
        self.pos_net = nn.Sequential(
            nn.Linear(self.pos_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU()
        )
        
        self.vel_net = nn.Sequential(
            nn.Linear(self.vel_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU()
        )
        
        self.orient_net = nn.Sequential(
            nn.Linear(self.orient_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU()
        )
        
        self.lidar_net = nn.Sequential(
            nn.Linear(self.lidar_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 96),
            nn.ReLU()
        )
        
        # Combine all features
        combined_dim = 64 + 64 + 32 + 96  # Sum of output dimensions from each network
        self.combine_net = nn.Sequential(
            nn.Linear(combined_dim, features_dim),
            nn.ReLU()
        )
        
    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # Split observations into components
        pos = observations[:, :self.pos_dim]
        vel = observations[:, self.pos_dim:self.pos_dim + self.vel_dim]
        orient = observations[:, self.pos_dim + self.vel_dim:self.pos_dim + self.vel_dim + self.orient_dim]
        lidar = observations[:, -self.lidar_dim:]
        
        # Process each component
        pos_features = self.pos_net(pos)
        vel_features = self.vel_net(vel)
        orient_features = self.orient_net(orient)
        lidar_features = self.lidar_net(lidar)
        
        # Combine features
        combined = torch.cat([
            pos_features,
            vel_features,
            orient_features,
            lidar_features
        ], dim=1)
        
        return self.combine_net(combined)

class DroneActorCriticPolicy(ActorCriticPolicy):
    """
    Custom actor-critic policy for drone navigation.
    Uses the custom feature extractor and separate policy and value networks.
    """
    
    def __init__(
        self,
        observation_space,
        action_space,
        lr_schedule,
        *args,
        **kwargs
    ):
        # Initialize with custom feature extractor
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            # Add custom network architecture
            features_extractor_class=DroneFeatureExtractor,
            features_extractor_kwargs=dict(features_dim=256),
            net_arch=[dict(pi=[128, 128], vf=[128, 128])],
            *args,
            **kwargs
        )
        
    def forward(self, obs: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass in the neural network
        """
        features = self.extract_features(obs)
        latent_pi, latent_vf = self.mlp_extractor(features)
        
        # Get values and distribution
        values = self.value_net(latent_vf)
        distribution = self._get_action_dist_from_latent(latent_pi)
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        
        return actions, values, log_prob 