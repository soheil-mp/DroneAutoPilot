<div align="center">

# 🚁 SkyPilot

### ROS2 Drone Navigation System with Reinforcement Learning

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

*An intelligent drone navigation system combining classical approaches with cutting-edge reinforcement learning*

[Features](#-features) •
[Installation](#%EF%B8%8F-installation) •
[Usage](#-usage) •
[Documentation](#-documentation) •
[Contributing](#-contributing)

</div>

---

## 🌟 Features

### Core Capabilities
- 🧠 Hybrid navigation combining classical and RL approaches
- 🎯 Deep reinforcement learning for optimal path planning
- 🚧 Real-time obstacle detection and avoidance
- 📱 Web-based mission control interface
- 🎮 Support for multiple drone platforms (PX4, ArduPilot)

### AI & Learning
- 🤖 PPO implementation using Stable Baselines3
- 🌍 Custom OpenAI Gym environment
- 📈 Experience replay & curriculum learning
- 🔄 Transfer learning from simulation to real hardware

### Interface & Control
- 🗺️ Interactive map with real-time tracking
- 📊 Live telemetry visualization
- ⚡ Battery and connection monitoring
- 🎯 Waypoint & geofence management

---

## 🛠️ System Requirements

### Hardware
| Component | Recommendation |
|-----------|---------------|
| Flight Controller | Pixhawk 4/6 |
| Onboard Computer | Raspberry Pi 4 / Jetson Nano |
| Sensors | GPS, IMU, Optional: Depth camera/LiDAR |
| Training Hardware | CUDA-capable GPU (Optional) |

### Software
| Requirement | Version |
|------------|---------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Python | 3.8+ |
| CUDA | 11.7+ |
| Node.js | 16+ |

---

## ⚡ Quick Start

### 1. Base Installation
```bash
# Install ROS2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop
```

### 2. Install Dependencies
```bash
# Python packages
pip install torch torchvision torchaudio
pip install stable-baselines3[extra]
pip install gym tensorboard pyyaml fastapi uvicorn

# MAVROS and PX4
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Node.js
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### 3. Clone & Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/soheil-mp/SkyPilot.git
cd ..
colcon build
source install/setup.bash
```

### 4. Setup Web Interface

```bash
# Frontend setup
cd ~/ros2_ws/src/SkyPilot/web/frontend
npm install

# Backend setup
cd ../backend
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt
```

### 5. Launch System

#### Training Mode
```bash
# Start simulation
ros2 launch drone_nav_rl gazebo_sim.launch.py

# Begin training
ros2 run drone_nav_rl train_agent.py

# Monitor training (optional)
tensorboard --logdir ./logs/training
```

#### Deployment Mode
```bash
# Configure drone
ros2 launch drone_nav_rl params_config.launch.py

# Start navigation stack
ros2 launch drone_nav_rl navigation.launch.py

# Launch mission control
ros2 launch drone_nav_rl mission_control.launch.py
```

#### Web Interface
```bash
# Terminal 1: Start backend server
cd web/backend
source myenv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000

# Terminal 2: Start frontend development server
cd web/frontend
npm start

# For production deployment
npm run build
```

---

## 🔧 Configuration

The system can be configured through YAML files in the `config` directory:

| File | Purpose |
|------|---------|
| `rl_params.yaml` | RL training hyperparameters |
| `navigation_params.yaml` | Navigation settings |
| `drone_params.yaml` | Drone hardware configuration |
| `mission_params.yaml` | Mission control parameters |

---

## 📁 Project Structure

```
SkyPilot/
├── 📂 config/          # Configuration files
├── 📂 launch/          # Launch files
├── 📂 models/          # Trained RL models
├── 📂 scripts/         # Training & evaluation
├── 📂 src/            # Core source code
├── 📂 web/            # Web interface
└── 📂 tests/          # Unit tests
```

---

## 📊 Performance Metrics

| Metric | Description |
|--------|-------------|
| Success Rate | Target reaching efficiency |
| Path Optimality | Navigation efficiency |
| Collision Rate | Safety performance |
| Response Latency | Real-time performance |

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

1. 🍴 Fork the repository
2. 🌿 Create a feature branch
3. ✍️ Commit your changes
4. 🚀 Push to the branch
5. 📬 Open a Pull Request

---

## 📚 Documentation

- [ROS2 Docs](https://docs.ros.org/en/humble/)
- [Stable Baselines3](https://stable-baselines3.readthedocs.io/)
- [PX4 Guide](https://dev.px4.io/)
- [FastAPI Docs](https://fastapi.tiangolo.com/)

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

Made with ❤️ by the SkyPilot Team

[⬆ Back to Top](#-skypilot)

</div>