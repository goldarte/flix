#!/usr/bin/env python3

"""Gazebo integration test for Flix external-odometry position control."""

import argparse
import errno
import math
import os
import pty
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from pymavlink.dialects.v20 import common as mavlink
from pyflix import Flix


@dataclass
class State:
    position: List[float]
    velocity: List[float]
    attitude: List[float]
    timestamp: float

    @property
    def yaw(self) -> float:
        w, x, y, z = self.attitude
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class GazeboOdometry:
    """Read the simulator ground-truth topic and forward it over MAVLink."""

    def __init__(self, master_uri: str, topic: str):
        self.master_uri = master_uri
        self.topic = topic
        self.state: Optional[State] = None
        self.error: Optional[BaseException] = None
        self.flix: Optional[Flix] = None
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._process: Optional[subprocess.Popen] = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        if shutil.which('gz') is None:
            raise RuntimeError('Gazebo Classic command-line client "gz" was not found')
        self._thread.start()

    def attach(self, flix: Flix):
        self.flix = flix

    def wait(self, timeout: float) -> State:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self.state is None and self.error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f'No position received from Gazebo topic {self.topic} via {self.master_uri}')
                self._condition.wait(remaining)
            if self.error is not None:
                raise RuntimeError('Gazebo position stream failed') from self.error
            assert self.state is not None
            if time.monotonic() - self.state.timestamp > timeout:
                raise TimeoutError(f'Gazebo position stream is stale on topic {self.topic}')
            return self.state

    def stop(self):
        self._stop.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
        self._thread.join(timeout=2)
        if self._process is not None and self._process.poll() is None:
            self._process.kill()

    def _run(self):
        env = os.environ.copy()
        env['GAZEBO_MASTER_URI'] = self.master_uri
        try:
            # Gazebo Classic buffers `gz topic` output when stdout is a pipe.
            # A pseudo-terminal makes every sample available as it is printed.
            master_fd, slave_fd = pty.openpty()
            try:
                self._process = subprocess.Popen(
                    ['gz', 'topic', '-e', self.topic],
                    stdin=subprocess.DEVNULL,
                    stdout=slave_fd,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=env,
                )
            finally:
                os.close(slave_fd)

            pose = {'position': {}, 'orientation': {}}
            section: Optional[str] = None
            previous: Optional[State] = None
            filtered_velocity = [0.0, 0.0, 0.0]
            with os.fdopen(master_fd, encoding='utf-8', errors='replace') as output:
                try:
                    for line in output:
                        if self._stop.is_set():
                            break
                        line = line.strip()
                        if line in ('position {', 'orientation {'):
                            section = line.split()[0]
                            continue
                        if line == '}':
                            section = None
                            continue
                        if ':' not in line:
                            continue
                        name, value = line.split(':', 1)
                        name = name.strip()
                        if section not in pose or name not in ('w', 'x', 'y', 'z'):
                            continue
                        try:
                            pose[section][name] = float(value.strip())
                        except ValueError:
                            continue
                        position_values = pose['position']
                        orientation_values = pose['orientation']
                        if not all(axis in position_values for axis in ('x', 'y', 'z')):
                            continue
                        if not all(axis in orientation_values for axis in ('w', 'x', 'y', 'z')):
                            continue

                        now = time.monotonic()
                        position = [
                            position_values['x'], position_values['y'], position_values['z']
                        ]
                        attitude = [
                            orientation_values['w'], orientation_values['x'],
                            orientation_values['y'], orientation_values['z']
                        ]
                        attitude_norm = math.sqrt(sum(component * component for component in attitude))
                        if attitude_norm <= 1e-6:
                            raise ValueError('Gazebo returned an invalid ground-truth quaternion')
                        attitude = [component / attitude_norm for component in attitude]
                        if previous is not None:
                            dt = now - previous.timestamp
                            if dt > 1e-4:
                                measured = [
                                    (position[i] - previous.position[i]) / dt for i in range(3)
                                ]
                                filtered_velocity = [
                                    filtered_velocity[i] * 0.6 + measured[i] * 0.4
                                    for i in range(3)
                                ]
                        state = State(position, filtered_velocity.copy(), attitude, now)
                        previous = state
                        pose = {'position': {}, 'orientation': {}}
                        with self._condition:
                            self.state = state
                            self._condition.notify_all()
                        if self.flix is not None:
                            self._send_odometry(state)
                except OSError as error:
                    # Linux PTYs report EIO after their slave process exits.
                    if error.errno != errno.EIO:
                        raise

            if not self._stop.is_set():
                assert self._process.stderr is not None
                detail = self._process.stderr.read().strip()
                raise RuntimeError(detail or 'gz topic exited unexpectedly')
        except BaseException as error:
            if not self._stop.is_set():
                with self._condition:
                    self.error = error
                    self._condition.notify_all()

    def _send_odometry(self, state: State):
        assert self.flix is not None
        unknown_covariance = [math.nan] + [0.0] * 20
        w, x, y, z = state.attitude
        body_velocity = world_to_body(state.velocity, state.attitude)
        self.flix.mavlink.odometry_send(
            time.monotonic_ns() // 1000,
            mavlink.MAV_FRAME_LOCAL_NED,
            mavlink.MAV_FRAME_BODY_FRD,
            state.position[0], -state.position[1], -state.position[2],
            [w, x, -y, -z],
            body_velocity[0], -body_velocity[1], -body_velocity[2],
            0.0, 0.0, 0.0,
            unknown_covariance,
            unknown_covariance,
            0,
            mavlink.MAV_ESTIMATOR_TYPE_VISION,
            100,
        )


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def angle_error(actual: float, target: float) -> float:
    return math.atan2(math.sin(actual - target), math.cos(actual - target))


def world_to_body(vector: Sequence[float], attitude: Sequence[float]) -> List[float]:
    """Rotate a local FLU vector into body FLU using a body-to-local quaternion."""
    w, x, y, z = attitude
    return [
        (1 - 2 * (y * y + z * z)) * vector[0] +
        2 * (x * y + w * z) * vector[1] +
        2 * (x * z - w * y) * vector[2],
        2 * (x * y - w * z) * vector[0] +
        (1 - 2 * (x * x + z * z)) * vector[1] +
        2 * (y * z + w * x) * vector[2],
        2 * (x * z + w * y) * vector[0] +
        2 * (y * z - w * x) * vector[1] +
        (1 - 2 * (x * x + y * y)) * vector[2],
    ]


def wait_for_rest(odometry: GazeboOdometry, timeout: float = 10.0) -> State:
    """Wait for the initially spawned model to settle onto the floor."""
    deadline = time.monotonic() + timeout
    settled_since: Optional[float] = None
    while time.monotonic() < deadline:
        state = odometry.wait(1.0)
        speed = math.sqrt(sum(component * component for component in state.velocity))
        if speed <= 0.05:
            if settled_since is None:
                settled_since = time.monotonic()
            elif time.monotonic() - settled_since >= 0.5:
                return state
        else:
            settled_since = None
        time.sleep(0.05)
    raise TimeoutError('Gazebo model did not settle before the position test')


def fly_to(flix: Flix, odometry: GazeboOdometry, target: List[float], yaw: float,
           tolerance: float, yaw_tolerance: float, timeout: float):
    print(
        f'Fly to: x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}, '
        f'yaw={math.degrees(yaw):.1f} deg'
    )
    deadline = time.monotonic() + timeout
    next_command = 0.0
    while time.monotonic() < deadline:
        if odometry.error is not None:
            raise RuntimeError('Gazebo odometry stream stopped') from odometry.error
        now = time.monotonic()
        if now >= next_command:
            flix.set_position(target, yaw=yaw)
            next_command = now + 0.5
        state = odometry.wait(1.0)
        speed = math.sqrt(sum(component * component for component in state.velocity))
        if (distance(state.position, target) <= tolerance and speed <= 0.25 and
                abs(angle_error(state.yaw, yaw)) <= yaw_tolerance):
            return
        time.sleep(0.05)
    state = odometry.wait(1.0)
    raise TimeoutError(
        f'Position timeout: target={target}, actual={state.position}, '
        f'error={distance(state.position, target):.2f} m, '
        f'yaw_error={math.degrees(angle_error(state.yaw, yaw)):.1f} deg')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gazebo-master', default='http://127.0.0.1:11345')
    parser.add_argument('--gazebo-topic', default='/gazebo/default/flix/ground_truth')
    parser.add_argument('--mavlink-device', default=os.getenv('FLIX_DEVICE'))
    parser.add_argument('--altitude', type=float, default=1.0)
    parser.add_argument('--side', type=float, default=0.75)
    parser.add_argument('--tolerance', type=float, default=0.15)
    parser.add_argument('--yaw-tolerance', type=float, default=10.0,
                        help='maximum yaw error at each waypoint, degrees')
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='timeout for each path segment, seconds')
    return parser.parse_args()


def main():
    args = parse_args()
    if (args.altitude <= 0 or args.side <= 0 or args.tolerance <= 0 or
            args.yaw_tolerance <= 0 or args.timeout <= 0):
        raise ValueError('Altitude, side, tolerances, and timeout must be positive')

    odometry = GazeboOdometry(args.gazebo_master, args.gazebo_topic)
    flix: Optional[Flix] = None
    odometry.start()
    try:
        odometry.wait(10.0)
        initial = wait_for_rest(odometry)
        print(f'Gazebo connected: {args.gazebo_master}')
        print(f'Initial position: {initial.position}')
        print(f'Held yaw: {math.degrees(initial.yaw):.1f} deg')

        flix = Flix(device=args.mavlink_device)
        odometry.attach(flix)
        time.sleep(0.6)  # allow the controller to receive several odometry samples
        flix.set_mode('AUTO')
        flix.set_armed(True)

        x, y, ground = initial.position
        altitude = ground + args.altitude
        yaw = initial.yaw
        yaw_tolerance = math.radians(args.yaw_tolerance)
        path = [
            [x, y, altitude],
            [x + args.side, y, altitude],
            [x + args.side, y + args.side, altitude],
            [x, y + args.side, altitude],
            [x, y, altitude],
            [x, y, ground],
        ]
        for target in path:
            fly_to(
                flix, odometry, target, yaw, args.tolerance, yaw_tolerance, args.timeout
            )

        flix.set_armed(False)
        print('Position-control Gazebo test passed')
    finally:
        if flix is not None and flix.armed:
            try:
                flix.set_armed(False)
            except Exception as error:
                print(f'Warning: failed to disarm during cleanup: {error}')
        odometry.stop()


if __name__ == '__main__':
    main()
