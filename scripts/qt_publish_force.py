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
        
        
        # Parameters for virtual hand motion
        self.k = 1000.0   # N/m, stiffness
        self.b = 10.0     # N*s/m, damping
        self.A = 0.1      # m, amplitude of hand motion
        self.freq = 0.2  # Hz, handshake frequency

        # Box entity
        self.entity = Entity()
        self.entity.name = "fr3::fr3_link7"
        self.entity.type = Entity.LINK

        # Track time
        self.start_time = time.time()
        self.x_box = 0.0   # Current box x-position (we can estimate or read from simulation if available)
        self.v_box = 0.0   # Box velocity along x
        
    def compute_force(self, t):
        # Virtual hand sinusoidal position
        x_hand = self.A * math.sin(2 * math.pi * self.freq * t)
        v_hand = 2 * math.pi * self.freq * self.A * math.cos(2 * math.pi * self.freq * t)

        # Spring-damper force
        delta_x = x_hand - self.x_box
        v_rel = v_hand - self.v_box
        F = self.k * delta_x + self.b * v_rel
        return F

    def publish_force(self):
        t = time.time() - self.start_time
        force = self.compute_force(t)
        msg = EntityWrench()
        msg.entity.name = self.entity.name
        msg.entity.type = self.entity.type
        msg.wrench.force.z = force
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandshakeForcePublisher()
    rate = 100
    dt = 1/rate
    while rclpy.ok():
        node.publish_force()
        time.sleep(dt)


if __name__ == '__main__':
    main()
