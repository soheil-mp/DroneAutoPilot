#!/bin/bash

# Deactivate conda if it's active
if [[ ! -z "${CONDA_DEFAULT_ENV}" ]]; then
    conda deactivate
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Create virtual environment with Python 3.10
if [ ! -d "myenv" ]; then
    python3.10 -m venv myenv --system-site-packages
    source myenv/bin/activate
    
    # Upgrade pip
    pip install --upgrade pip
    
    # Install build dependencies
    pip install wheel setuptools

    # Install requirements
    pip install -r requirements.txt
else
    source myenv/bin/activate
fi

# Add ROS2 Python paths
export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/drone_nav_rl_interfaces/local/lib/python3.10/dist-packages

# Run the server
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload 