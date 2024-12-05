# ROS2 Drone Navigation System with Reinforcement Learning

## Overview
This repository contains a ROS2 (Humble) based drone navigation system that combines classical navigation approaches with reinforcement learning (RL) for enhanced autonomous capabilities. The system provides intelligent path planning, adaptive obstacle avoidance, and sophisticated mission control functionalities using deep RL algorithms.

## 🚁 Features
- Autonomous navigation using hybrid classical and RL-based approaches
- Deep reinforcement learning for optimal path planning
- Real-time obstacle detection and dynamic avoidance
- Adaptive mission planning and execution
- Support for multiple drone platforms (PX4, ArduPilot)
- Advanced telemetry monitoring and data logging
- Simulation support with Gazebo and AirSim
- Web-based mission control interface
- Training environments for RL agents

## 🧠 Reinforcement Learning Components
- PPO (Proximal Policy Optimization) implementation using Stable Baselines3
- Custom OpenAI Gym environment for drone navigation
- Reward shaping for efficient and safe navigation
- Experience replay buffer for improved learning
- Transfer learning support from simulation to real hardware
- Curriculum learning for progressive skill acquisition

## 📋 Prerequisites

### Hardware Requirements
- Compatible drone/UAV platform
- Flight controller (Pixhawk 4/6 recommended)
- Onboard computer (Raspberry Pi 4, Jetson Nano, or similar)
- GPS module
- IMU sensor
- Optional: Depth camera or LiDAR for obstacle detection
- Optional: GPU for accelerated RL training

### Software Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.8+
- CUDA 11.7+ (for GPU training)
- PX4 or ArduPilot firmware
- MAVROS
- Gazebo and AirSim simulators
- PyTorch 2.0+
- Stable Baselines3
- OpenAI Gym

## 🛠️ Installation

1. Install ROS2 Humble
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

2. Install Python dependencies
```bash
pip install torch torchvision torchaudio
pip install stable-baselines3[extra]
pip install gym tensorboard
```

3. Install MAVROS and PX4
```bash
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

4. Clone and build this repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yourusername/drone_navigation_rl.git
cd ..
colcon build
source install/setup.bash
```

## 🎮 Usage

### Training the RL Agent
```bash
# Launch simulation
ros2 launch drone_nav_rl gazebo_sim.launch.py

# Start training
ros2 run drone_nav_rl train_agent.py

# Monitor training
tensorboard --logdir ./logs/training
```

### Real-world Deployment
```bash
# Configure drone parameters
ros2 launch drone_nav_rl params_config.launch.py

# Start navigation stack
ros2 launch drone_nav_rl navigation.launch.py

# Launch mission control
ros2 launch drone_nav_rl mission_control.launch.py
```

## 📁 Project Structure
```
drone_navigation_rl/
├── config/                 # Configuration files
├── launch/                 # Launch files
├── models/                 # Trained RL models
├── scripts/               
│   ├── train_agent.py     # Training script
│   └── evaluate_agent.py   # Evaluation script
��── src/
│   ├── environments/      # Custom Gym environments
│   ├── networks/          # Neural network architectures
│   └── utilities/         # Helper functions
└── tests/                 # Unit tests
```

## 🔧 Configuration
Configure the system through YAML files in the `config` directory:
- `rl_params.yaml`: RL training hyperparameters
- `navigation_params.yaml`: Navigation parameters
- `drone_params.yaml`: Drone configurations

## 📊 Performance Metrics
- Success rate in reaching targets
- Average episode reward
- Collision rate
- Path optimality
- Training convergence
- Battery efficiency

## 🤝 Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 📝 License
This project is licensed under the MIT License - see the LICENSE file for details.

## 📚 References
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Stable Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [PX4 Developer Guide](https://dev.px4.io/)
- [OpenAI Gym Documentation](https://gym.openai.com/) 