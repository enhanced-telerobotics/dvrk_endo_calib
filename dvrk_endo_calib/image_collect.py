import argparse
from collections import deque
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time

from crtk_msgs.msg import OperatingState
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty
import yaml

from .workspace import (
    JOINT_TOLERANCES, Workspace, at_position, config_dir, latest_workspace, positive_float, stamp_ns,
    timestamp, valid_position,
)


EYES = ('left', 'right')
IMAGE_TOPICS = {
    eye: f'/stereo_source/{eye}/camera/image_raw/compressed' for eye in EYES
}
DEFAULT_CHECKERBOARD = (12, 9)
# The lab's Si ECM can settle roughly 0.01 rad off its roll setpoint.
CAPTURE_JOINT_TOLERANCES = (*JOINT_TOLERANCES[:3], 0.02)
CHECKERBOARD_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
)


class UnusableCapture(RuntimeError):
    """The ECM is idle, but this pose cannot produce an image pair."""


class ImageCollector(Node):
    def __init__(self, workspace, max_skew):
        super().__init__('image_collect')
        self.workspace = workspace
        self.max_skew_ns = round(max_skew * 1_000_000_000)
        self.state = None
        self.joints = None
        self.joints_received_at = 0.0
        self.motion_pending = False
        self.move_stamp_ns = 0
        self.saw_busy = False
        self.frames = {eye: deque(maxlen=8) for eye in EYES}
        self.pair = None
        self.capture_after_ns = 0

        arm = workspace.arm
        self.state_subscription = self.create_subscription(
            OperatingState, f'{arm}/operating_state', self.on_state,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState, f'{arm}/measured_js', self.on_joints, qos_profile_sensor_data,
        )
        self.move = self.create_publisher(JointState, f'{arm}/move_jp', 10)
        self.hold = self.create_publisher(Empty, f'{arm}/hold', 10)
        self.cameras = {
            eye: self.create_subscription(
                CompressedImage, topic, lambda msg, eye=eye: self.on_image(eye, msg),
                qos_profile_sensor_data,
            )
            for eye, topic in IMAGE_TOPICS.items()
        }

    def on_state(self, message):
        stamp = stamp_ns(message.header)
        if self.motion_pending and stamp and stamp < self.move_stamp_ns:
            return
        self.state = message
        if self.motion_pending and message.is_busy:
            self.saw_busy = True

    def on_joints(self, message):
        self.joints = message
        self.joints_received_at = time.monotonic()

    def on_image(self, eye, message):
        stamp = stamp_ns(message.header)
        if self.pair is not None or stamp <= self.capture_after_ns or not message.data:
            return

        other_eye = 'right' if eye == 'left' else 'left'
        other = min(self.frames[other_eye], default=None,
                    key=lambda frame: abs(stamp_ns(frame.header) - stamp))
        if other is not None and abs(stamp_ns(other.header) - stamp) <= self.max_skew_ns:
            self.pair = (message, other) if eye == 'left' else (other, message)
            for frames in self.frames.values():
                frames.clear()
        else:
            self.frames[eye].append(message)

    def joints_are_fresh(self):
        return self.joints is not None and time.monotonic() - self.joints_received_at < 1.0

    def check_joint_order(self):
        if not valid_position(self.joints.position):
            raise RuntimeError('ECM feedback must contain four finite joint positions.')
        names = self.workspace.joint_names
        if names and list(self.joints.name) != names:
            raise RuntimeError('ECM joint names or order differ from the recorded workspace.')

    def check_arm_state(self):
        if not self.state_subscription.get_publisher_count():
            raise RuntimeError('Lost the ECM operating-state publisher.')
        if self.state.state != 'ENABLED' or not self.state.is_homed:
            raise RuntimeError('ECM must be enabled and homed.')

    def wait_until(self, condition, timeout, failure):
        deadline = time.monotonic() + timeout
        while not condition():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(failure)
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def connect(self, timeout):
        self.wait_until(
            lambda: self.state is not None and self.joints_are_fresh()
            and self.move.get_subscription_count() and self.hold.get_subscription_count(),
            timeout, f'No complete ECM feedback and command connection for {self.workspace.arm}.',
        )
        self.start_capture()
        self.wait_until(lambda: self.pair is not None, timeout,
                        'No timestamp-matched stereo frames on the two compressed image topics.')
        for image in self.pair:
            image_extension(image)
        self.wait_until(
            lambda: self.state.state == 'ENABLED' and self.state.is_homed
            and not self.state.is_busy and self.joints_are_fresh(),
            timeout, 'ECM must be enabled, homed, and idle with current joint feedback.',
        )
        self.check_arm_state()
        self.check_joint_order()

    def move_and_settle(self, target, settle, timeout):
        ready_at = time.monotonic()
        self.wait_until(lambda: self.joints_received_at > ready_at, timeout,
                        'No current ECM joint feedback before moving.')
        self.check_arm_state()
        if self.state.is_busy:
            raise RuntimeError('ECM must be idle before moving.')
        self.check_joint_order()
        if not self.move.get_subscription_count() or not self.hold.get_subscription_count():
            raise RuntimeError('Lost the ECM motion command connection.')

        sent_at = time.monotonic()
        self.move_stamp_ns = self.get_clock().now().nanoseconds
        self.saw_busy = False
        self.motion_pending = True
        self.move.publish(JointState(position=list(target)))

        stationary_since = None

        def feedback_after_command():
            return self.joints_are_fresh() and self.joints_received_at > sent_at

        def settled():
            nonlocal stationary_since
            self.check_arm_state()
            if not self.saw_busy or self.state.is_busy or not feedback_after_command():
                stationary_since = None
                return False
            self.check_joint_order()
            if not at_position(self.joints.position, target, CAPTURE_JOINT_TOLERANCES):
                stationary_since = None
                return False
            now = time.monotonic()
            if stationary_since is None:
                stationary_since = now
            return now - stationary_since >= settle

        try:
            self.wait_until(settled, timeout, 'ECM motion timed out.')
        except TimeoutError as error:
            self.check_arm_state()
            fresh_feedback = feedback_after_command()
            details = self.move_timeout_details(target, fresh_feedback)
            if self.state.is_busy or not self.saw_busy or not fresh_feedback:
                raise TimeoutError(details) from error

            self.motion_pending = False
            self.get_logger().warning(f'{details} Skipping this pose.')
            return False

        self.motion_pending = False
        return True

    def move_timeout_details(self, target, fresh_feedback):
        if self.state.is_busy:
            status = 'controller still reports busy'
        elif self.saw_busy:
            status = 'controller reported idle after motion'
        else:
            status = 'no busy acknowledgment after move_jp'

        message = f'ECM did not reach and settle at {target}. {status.capitalize()}.'
        if not fresh_feedback:
            return f'{message} No fresh measured joint feedback.'

        self.check_joint_order()
        measured = list(self.joints.position)
        errors = ', '.join(f'{position - desired:+.6f}'
                           for position, desired in zip(measured, target))
        return (f'{message} Measured joints {measured}. '
                f'Measured minus target ({errors}) in rad, rad, m, rad.')

    def start_capture(self):
        self.capture_after_ns = self.get_clock().now().nanoseconds
        self.pair = None
        for frames in self.frames.values():
            frames.clear()

    def capture(self, target, timeout):
        self.start_capture()

        def pair_is_ready():
            self.check_arm_state()
            if self.state.is_busy or not self.joints_are_fresh():
                raise RuntimeError('ECM stopped reporting idle joint feedback during capture.')
            self.check_joint_order()
            if not at_position(self.joints.position, target, CAPTURE_JOINT_TOLERANCES):
                raise UnusableCapture('ECM left the target position during capture.')
            return self.pair is not None

        self.wait_until(pair_is_ready, timeout,
                        'No fresh timestamp-matched stereo pair after the ECM settled.')
        return self.pair

    def stop_pending_move(self):
        if self.motion_pending and rclpy.ok():
            self.hold.publish(Empty())
            if not self.hold.wait_for_all_acked(Duration(seconds=0.25)):
                self.get_logger().warning('No acknowledgment for the ECM hold command.')


def image_extension(image):
    signature = bytes(image.data[:8])
    if signature.startswith(b'\xff\xd8\xff'):
        return '.jpg'
    if signature == b'\x89PNG\r\n\x1a\n':
        return '.png'
    if signature.startswith((b'II*\x00', b'MM\x00*')):
        return '.tiff'
    raise ValueError(f'Unsupported compressed image format: {image.format!r}')


def checkerboard_visible(image, pattern_size):
    try:
        pixels = cv2.imdecode(np.frombuffer(image.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if pixels is None:
            return False
        found, _ = cv2.findChessboardCorners(pixels, pattern_size, CHECKERBOARD_FLAGS)
        return found
    except cv2.error:
        return False


def write_yaml(path, data):
    with path.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)


def save_pair(output, name, pair):
    with TemporaryDirectory(prefix='.pair-', dir=output) as temporary_dir:
        transfers = []
        for eye, image in zip(EYES, pair):
            extension = image_extension(image)
            staged = Path(temporary_dir) / (eye + extension)
            staged.write_bytes(bytes(image.data))
            transfers.append((staged, output / eye / (name + extension)))

        try:
            for staged, destination in transfers:
                staged.rename(destination)
        except BaseException:
            # Roll back a partially published pair, including on Ctrl+C.
            for staged, destination in transfers:
                if not staged.exists():
                    destination.unlink(missing_ok=True)
            raise


def main():
    parser = argparse.ArgumentParser(description='Move the ECM and capture stereo image pairs.')
    parser.add_argument('-n', '--count', type=int, default=100,
                        help='Number of valid stereo pairs to save (default: 100)')
    parser.add_argument('--workspace', type=Path, help='Workspace JSON (default: latest saved)')
    parser.add_argument('--output-dir', type=Path, default=config_dir() / 'captures',
                        help='Capture parent directory (default: %(default)s)')
    parser.add_argument('--checkerboard', type=int, nargs=2, metavar=('COLS', 'ROWS'),
                        default=DEFAULT_CHECKERBOARD,
                        help='Checkerboard squares across and down (default: 12 9)')
    parser.add_argument('--square-size-mm', type=positive_float, default=5.0,
                        help='Checkerboard square side length in millimeters (default: 5)')
    parser.add_argument('--settle', type=positive_float, default=1.0,
                        help='Stationary time before capture in seconds (default: 1.0)')
    parser.add_argument('--max-skew', type=positive_float, default=0.02,
                        help='Maximum stereo timestamp difference in seconds (default: 0.02)')
    parser.add_argument('--timeout', type=positive_float, default=20.0,
                        help='Timeout per connection, motion or capture in seconds (default: 20)')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.count <= 0:
        parser.error('--count must be positive')
    cols, rows = args.checkerboard
    if min(cols, rows) < 4:
        parser.error('--checkerboard needs at least 4 squares along each dimension')
    if args.settle >= args.timeout:
        parser.error('--timeout must exceed --settle')

    pattern_size = (cols - 1, rows - 1)
    checkerboard = {
        'square_size_mm': args.square_size_mm,
        'squares': {'columns': cols, 'rows': rows},
        'inner_corners': {'columns': pattern_size[0], 'rows': pattern_size[1]},
    }

    max_attempts = 3 * args.count // 2
    filename_width = max(4, len(str(args.count)))
    try:
        workspace_path = (args.workspace or latest_workspace()).expanduser().resolve()
        workspace = Workspace.load(workspace_path)
        targets = workspace.targets(max_attempts)
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ImageCollector(workspace, args.max_skew)
    output = args.output_dir.expanduser().resolve() / timestamp()
    saved = 0
    attempts = 0
    result = 0
    try:
        node.get_logger().info(f'Using {workspace_path}. Waiting for ECM and stereo topics.')
        node.connect(args.timeout)
        output.mkdir(parents=True)
        for eye in EYES:
            (output / eye).mkdir()
        write_yaml(output / 'run.yaml', {
            'workspace': str(workspace_path),
            'arm_namespace': workspace.arm,
            'joint_names': list(node.joints.name),
            'initial_pose': {
                'measured_joint_positions': list(node.joints.position),
                'measured_joint_stamp_ns': stamp_ns(node.joints.header),
            },
            'requested_pairs': args.count,
            'max_attempts': max_attempts,
            'checkerboard': checkerboard,
            'image_topics': {eye: camera.topic_name for eye, camera in node.cameras.items()},
            'settle_sec': args.settle,
            'max_skew_sec': args.max_skew,
            'capture_joint_tolerances': dict(zip(
                ('yaw_rad', 'pitch_rad', 'insertion_m', 'roll_rad'), CAPTURE_JOINT_TOLERANCES,
            )),
        })
        node.get_logger().info(f'Saving capture run to {output}')
        for attempts, target in enumerate(targets, start=1):
            node.get_logger().info(
                f'Attempt {attempts}/{max_attempts} ({saved}/{args.count} pairs saved)'
            )
            if not node.move_and_settle(target, args.settle, args.timeout):
                continue
            try:
                pair = node.capture(target, args.timeout)
            except (TimeoutError, UnusableCapture) as error:
                node.get_logger().warning(f'Skipped attempt {attempts}. {error}')
                continue
            if not all(checkerboard_visible(image, pattern_size) for image in pair):
                node.get_logger().warning(
                    f'Skipped attempt {attempts}. Checkerboard detection failed for the stereo pair.'
                )
                continue
            save_pair(output, f'{saved + 1:0{filename_width}d}', pair)
            saved += 1
            if saved == args.count:
                break
    except KeyboardInterrupt:
        result = 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f'Image collection stopped: {error}', file=sys.stderr)
        result = 1
    finally:
        try:
            node.stop_pending_move()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    if output.is_dir():
        print(f'Saved {saved}/{args.count} validated stereo pairs in {attempts} attempts to {output}')
        if result == 0 and saved < args.count:
            print(f'Reached the {max_attempts}-attempt limit.', file=sys.stderr)
            result = 1
    return result
