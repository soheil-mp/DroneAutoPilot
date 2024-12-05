#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from skypilot.drone_controller import DroneController

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
