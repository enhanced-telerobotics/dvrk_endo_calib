import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
from tempfile import TemporaryDirectory


WORKSPACE_VERSION = 1
# ECM joint order is yaw, pitch, insertion, roll. The units are rad, rad, m, rad.
JOINT_TOLERANCES = (0.005, 0.005, 0.001, 0.005)


def config_dir():
    return Path.home() / '.config' / 'dvrk_endo_calib'


def timestamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('expected a positive finite number')
    return number


def valid_position(position):
    return (
        len(position) == len(JOINT_TOLERANCES)
        and all(math.isfinite(value) for value in position)
    )


def at_position(position, target, tolerances=JOINT_TOLERANCES):
    return all(
        abs(actual - desired) <= tolerance
        for actual, desired, tolerance in zip(position, target, tolerances)
    )


def write_json(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')


@dataclass
class Workspace:
    arm: str
    joint_names: list[str]
    samples: list[dict]

    @classmethod
    def load(cls, path):
        with path.open(encoding='utf-8') as stream:
            data = json.load(stream)
        try:
            if data['version'] != WORKSPACE_VERSION:
                raise ValueError('unsupported version')
            arm = data['arm_namespace']
            if not isinstance(arm, str) or not arm.startswith('/') or arm.endswith('/'):
                raise ValueError('expected an absolute arm namespace such as /ECM')
            names = data['joint_names']
            if not isinstance(names, list) or (
                names and (len(names) != len(JOINT_TOLERANCES)
                           or not all(isinstance(name, str) for name in names))
            ):
                raise ValueError('joint names must match the four ECM joints')
            samples = data['samples']
            if not isinstance(samples, list) or not all(
                valid_position(sample['position']) for sample in samples
            ):
                raise ValueError('expected samples with four finite ECM joint positions')
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(f'Invalid workspace {path}: {error}') from error
        return cls(arm, names, samples)

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(prefix='.workspace-', dir=path.parent) as directory:
            temporary_path = Path(directory) / path.name
            write_json(temporary_path, {
                'version': WORKSPACE_VERSION,
                'arm_namespace': self.arm,
                'joint_names': self.joint_names,
                'samples': self.samples,
            })
            temporary_path.rename(path)

    def targets(self, count):
        positions = list(dict.fromkeys(tuple(sample['position']) for sample in self.samples))
        if len(positions) < 2:
            raise ValueError('Record at least two distinct ECM waypoints with workspace_collect.')

        targets = []
        while len(targets) < count:
            random.shuffle(positions)
            if targets and positions[0] == targets[-1]:
                positions[0], positions[1] = positions[1], positions[0]
            targets.extend(positions[:count - len(targets)])
        return targets


def latest_workspace():
    path = max(config_dir().glob('workspace_*.json'), default=None)
    if path is None:
        raise FileNotFoundError(
            f'No workspace found in {config_dir()}. Run workspace_collect first.'
        )
    return path
