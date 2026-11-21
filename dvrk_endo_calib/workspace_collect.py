import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from .workspace import (
    Workspace, at_position, config_dir, positive_float, stamp_ns, timestamp, valid_position,
)


class WorkspaceCollector(Node):
    def __init__(self, arm, rate):
        super().__init__('workspace_collect')
        self.workspace = Workspace(arm, [], [])
        self.sample_period = 1.0 / rate
        self.next_sample_at = 0.0
        self.joints = self.create_subscription(
            JointState, f'{arm}/measured_js', self.record, qos_profile_sensor_data,
        )

    def record(self, message):
        now = time.monotonic()
        if now < self.next_sample_at or not valid_position(message.position):
            return

        positions = list(message.position)
        names = list(message.name)
        if names and len(names) != len(positions):
            return

        samples = self.workspace.samples
        if not samples:
            self.workspace.joint_names = names
            self.get_logger().info('Recording. Hand-move the ECM, then press Ctrl+C to save.')
        elif names != self.workspace.joint_names or at_position(positions, samples[-1]['position']):
            return

        samples.append({'stamp_ns': stamp_ns(message.header), 'position': positions})
        self.next_sample_at = now + self.sample_period


def main():
    parser = argparse.ArgumentParser(description='Record a hand-guided ECM joint workspace.')
    parser.add_argument('--arm', default='/ECM', help='dVRK ECM namespace (default: /ECM)')
    parser.add_argument('--rate', type=positive_float, default=10.0,
                        help='Maximum number of samples per second (default: 10)')
    args = parser.parse_args(remove_ros_args()[1:])
    arm = '/' + args.arm.strip('/')
    if arm == '/':
        parser.error('--arm must name an ECM namespace')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WorkspaceCollector(arm, args.rate)
    try:
        node.get_logger().info(f'Waiting for {node.joints.topic_name} ...')
        deadline = time.monotonic() + 10.0
        while not node.workspace.samples:
            if time.monotonic() >= deadline:
                raise TimeoutError(f'No valid ECM joint feedback on {node.joints.topic_name}.')
            rclpy.spin_once(node, timeout_sec=0.1)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except TimeoutError as error:
        print(error, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    count = len(node.workspace.samples)
    if count == 0:
        print('No ECM waypoints recorded.', file=sys.stderr)
        return 1
    path = config_dir() / f'workspace_{timestamp()}.json'
    try:
        node.workspace.save(path)
    except OSError as error:
        print(f'Could not save workspace: {error}', file=sys.stderr)
        return 1
    print(f'Saved {count} ECM waypoints to {path}')
    return 0
