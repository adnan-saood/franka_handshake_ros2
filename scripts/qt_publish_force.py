#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench, Entity
import math
import time


class HandshakeForcePublisher(Node):
    def __init__(self):
        super().__init__('handshake_force_publisher')
        self.pub = self.create_publisher(EntityWrench, '/world/empty/wrench', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.01, self.publish_force)  # 100 Hz

    def publish_force(self):
        t = time.time() - self.start_time
        force = 500.0 * math.sin(2 * math.pi * 0.5 * t)  # 5N, 0.5Hz
        msg = EntityWrench()
        msg.entity.name = 'box'
        msg.entity.type = Entity.MODEL
        msg.wrench.force.y = force
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandshakeForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
