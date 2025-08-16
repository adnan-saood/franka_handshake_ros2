#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench, Entity
import math
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped


class HandshakeForcePublisher(Node):
    def __init__(self):
        super().__init__('handshake_force_publisher')
        self.pub = self.create_publisher(EntityWrench, '/world/empty/wrench', 10)

        # Parameters for virtual hand motion
        self.k = 100.0   # N/m, stiffness
        self.b = 0.0     # N*s/m, damping
        self.A = 0.1      # m, amplitude of hand motion
        self.freq = 0.65  # Hz, handshake frequency

        # Box entity
        self.entity = Entity()
        self.entity.name = "fr3::fr3_link7"
        self.entity.type = Entity.LINK

        # Track time
        self.start_time = time.time()
        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_force)
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.z_link = 0.0  # Initial position of the box
        self.dz_link = 0.0  # Initial velocity of the box
        self.z_hand = 0.0  # Initial position of the virtual hand
        self.dz_hand = 0.0  # Initial velocity of the virtual hand
        self.z_hand_initial = 0.0


    def init_args(self):
        # try to get initial z_hand as to be matching robot z_link
        while self.get_link_pose()[2] == 0:
            print("Waiting for link pose...")
        self.z_link = self.get_link_pose()[2]
        self.z_hand = self.z_link
        self.z_hand_initial = self.z_hand  # Store initial position of the hand

    def compute_force(self, t):
        # Virtual hand sinusoidal position
        self.z_hand = self.z_hand_initial + self.A * math.sin(2 * math.pi * self.freq * t)
        self.dz_hand = 2 * math.pi * self.freq * self.A * math.cos(2 * math.pi * self.freq * t)

        # Spring-damper force
        self.z_link = self.get_link_pose()[2]
        if self.z_link == 0:
            return 0
        delta_z = self.z_hand - self.z_link
        v_rel = self.dz_hand - self.dz_link
        F = self.k * delta_z + self.b * v_rel
        return F

    def publish_force(self):
        t = time.time() - self.start_time
        if self.z_hand_initial == 0:
            pose = self.get_link_pose()
            if pose[2] != 0:
                self.z_link = pose[2]
                self.z_hand = self.z_link
                self.z_hand_initial = self.z_hand
                self.get_logger().info(f"Initialized z_hand_initial: {self.z_hand_initial}")
                self.start_time = time.time()
            else:
                self.get_logger().info("Waiting for link pose...")
            return
        force = self.compute_force(t)
        if force == 0:
            return
        msg = EntityWrench()
        msg.entity.name = self.entity.name
        msg.entity.type = self.entity.type
        msg.wrench.force.z = force
        self.pub.publish(msg)

    def get_link_pose(self):
        try:
            now = rclpy.time.Time()
            if self.tf_buffer.can_transform('fr3_link0', 'fr3_link7', now, timeout=rclpy.duration.Duration(seconds=0.5)):
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    'fr3_link0', 'fr3_link7', now)
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                z = trans.transform.translation.z
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w
                return (x, y, z, qx, qy, qz, qw)
            else:
                print("Transform not available")
                return (0, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            print(f"Could not get transform: {e}")
            return (0, 0, 0, 0, 0, 0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = HandshakeForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
