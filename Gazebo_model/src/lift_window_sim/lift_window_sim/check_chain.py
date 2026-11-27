"""Sjekker at de fire teleskoptrinnene beveger seg sammen.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Check(Node):
    def __init__(self):
        super().__init__('lift_check_chain')
        self.declare_parameter('minimum_extension',.5)
        self.sample = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)

    def on_state(self, msg):
        data = dict(zip(msg.name, msg.position))
        joints = ('boom_extension',) + tuple(f'stage{i}_slide' for i in range(2,5))
        if all(name in data for name in joints):
            self.sample = [float(data[name]) for name in joints]


def main():
    rclpy.init()
    node = Check()
    try:
        minimum=float(node.get_parameter('minimum_extension').value)
        deadline = time.monotonic() + 20
        # Jeg brukte KI til å utforme og feilsøke sjekken av om alle trinnene følger uttrekket.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            if node.sample is not None and node.sample[0] > minimum and \
                    max(abs(v-node.sample[0]) for v in node.sample[1:]) <= .03:
                break
        if node.sample is None:
            raise RuntimeError('Ingen målte stillinger for teleskopleddene; kan ikke kontrollere armfestet')
        err = [abs(v-node.sample[0]) for v in node.sample[1:]]
        if node.sample[0] < minimum or max(err) > .03:
            raise RuntimeError(f'Teleskopleddene følger ikke løftet: stillinger={node.sample}, avvik={err}')
        node.get_logger().info(f'CHAIN READY: alle fire trinn følger sammen; stillinger={node.sample}')
    except Exception as exc:
        node.get_logger().error(str(exc))
        sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
