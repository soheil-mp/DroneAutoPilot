#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from skypilot.mission_visualization import MissionVisualization

def main(args=None):
    rclpy.init(args=args)
    node = MissionVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
