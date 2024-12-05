import os
import torch
import torch.nn as nn
from typing import Dict, Optional, Type
from stable_baselines3 import PPO
from stable_baselines3.common.policies import BasePolicy
from .policy_network import DroneActorCriticPolicy

def init_weights(module: nn.Module) -> None:
    """Initialize neural network weights using orthogonal initialization"""
    if isinstance(module, (nn.Linear, nn.Conv2d)):
        nn.init.orthogonal_(module.weight.data, gain=1.0)
        if module.bias is not None:
            module.bias.data.fill_(0.0)

def create_policy(
    observation_space,
    action_space,
    lr_schedule,
    policy_kwargs: Optional[Dict] = None
) -> BasePolicy:
    """Create a new policy with custom architecture"""
    if policy_kwargs is None:
        policy_kwargs = {}
    
    # Use our custom policy by default
    policy = DroneActorCriticPolicy(
        observation_space=observation_space,
        action_space=action_space,
        lr_schedule=lr_schedule,
        **policy_kwargs
    )
    
    # Initialize weights
    policy.apply(init_weights)
    
    return policy

def load_policy(
    path: str,
    observation_space,
    action_space,
    device: str = "auto"
) -> BasePolicy:
    """Load a saved policy from disk"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"No policy found at {path}")
    
    # Load the saved model
    model = PPO.load(path, device=device)
    
    # Extract the policy
    policy = model.policy
    
    # Ensure the policy is compatible with the current spaces
    assert policy.observation_space == observation_space, "Observation space mismatch"
    assert policy.action_space == action_space, "Action space mismatch"
    
    return policy

def save_policy(policy: BasePolicy, path: str) -> None:
    """Save a policy to disk"""
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(path), exist_ok=True)
    
    # Save the policy state dict
    torch.save(policy.state_dict(), path)

def get_activation_function(name: str) -> Type[nn.Module]:
    """Get activation function by name"""
    activations = {
        "relu": nn.ReLU,
        "tanh": nn.Tanh,
        "sigmoid": nn.Sigmoid,
        "leaky_relu": nn.LeakyReLU,
        "elu": nn.ELU
    }
    
    if name.lower() not in activations:
        raise ValueError(f"Unknown activation function: {name}")
    
    return activations[name.lower()]

def create_mlp(
    input_dim: int,
    output_dim: int,
    net_arch: list,
    activation_fn: Type[nn.Module] = nn.ReLU,
    squash_output: bool = False
) -> nn.Module:
    """Create a multi-layer perceptron"""
    if len(net_arch) == 0:
        return nn.Linear(input_dim, output_dim)
    
    layers = []
    layers.append(nn.Linear(input_dim, net_arch[0]))
    layers.append(activation_fn())
    
    for idx in range(len(net_arch) - 1):
        layers.append(nn.Linear(net_arch[idx], net_arch[idx + 1]))
        layers.append(activation_fn())
    
    layers.append(nn.Linear(net_arch[-1], output_dim))
    
    if squash_output:
        layers.append(nn.Tanh())
    
    return nn.Sequential(*layers) 