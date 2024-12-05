#!/usr/bin/env python3

import unittest
import sys
import os
import subprocess
import time
import signal
import rclpy
import pytest

def start_test_environment():
    """Start the test environment"""
    # Get the directory containing this script
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Start the test launch file
    launch_file = os.path.join(tests_dir, 'fixtures', 'test_launch.py')
    launch_process = subprocess.Popen(
        ['ros2', 'launch', launch_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    # Wait for environment to initialize
    time.sleep(10)
    
    return launch_process

def stop_test_environment(launch_process):
    """Stop the test environment"""
    # Send SIGINT to the launch process
    launch_process.send_signal(signal.SIGINT)
    
    # Wait for process to terminate
    launch_process.wait(timeout=10)
    
    # Kill any remaining ROS processes
    subprocess.run(['pkill', '-f', 'ros2'])
    subprocess.run(['pkill', '-f', 'gazebo'])

def run_integration_tests():
    """Run integration tests"""
    # Get the directory containing this script
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Add the project root to Python path
    project_root = os.path.dirname(tests_dir)
    sys.path.insert(0, project_root)
    
    try:
        # Start the test environment
        launch_process = start_test_environment()
        
        # Initialize ROS
        rclpy.init()
        
        # Run the integration tests
        loader = unittest.TestLoader()
        start_dir = os.path.join(tests_dir, 'integration')
        suite = loader.discover(start_dir, pattern='test_*.py')
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Cleanup
        rclpy.shutdown()
        stop_test_environment(launch_process)
        
        return result.wasSuccessful()
        
    except Exception as e:
        print(f"Error running integration tests: {e}")
        return False
    
    finally:
        # Ensure cleanup happens even if tests fail
        try:
            rclpy.shutdown()
        except:
            pass
            
        try:
            stop_test_environment(launch_process)
        except:
            pass

if __name__ == '__main__':
    success = run_integration_tests()
    sys.exit(0 if success else 1) 