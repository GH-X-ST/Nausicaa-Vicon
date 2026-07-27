"""Read synchronized full rigid-body states from a Vicon DataStream server."""

import argparse
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

from vicon_dssdk import ViconDataStream


DEFAULT_HOST = "192.168.0.100:801"
DEFAULT_DERIVATIVE_CUTOFF_HZ = 8.0
_Vector3: TypeAlias = tuple[float, float, float]
_Quaternion: TypeAlias = tuple[float, float, float, float]
_FloatArray: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True)
class RigidBodyState:
    """Full pose and motion state for one visible Vicon subject."""

    position_m: _Vector3
    euler_xyz_rad: _Vector3
    quaternion_xyzw: _Quaternion
    velocity_world_m_s: _Vector3
    velocity_body_m_s: _Vector3
    angular_velocity_body_rad_s: _Vector3
    motion_valid: bool


@dataclass(frozen=True)
class ViconFrame:
    """One synchronized Vicon frame containing all visible subject states."""

    frame_number: int
    estimated_capture_time_s: float
    frame_rate_hz: float
    latency_s: float
    states: dict[str, RigidBodyState]
    occluded_subjects: tuple[str, ...]


@dataclass
class _MotionHistory:
    frame_number: int
    position_m: _FloatArray
    rotation_body_to_world: _FloatArray
    velocity_world_m_s: _FloatArray
    angular_velocity_world_rad_s: _FloatArray


class ViconTracker:
    """Track multiple Vicon subjects and estimate their full motion states.

    When ``subject_names`` is ``None``, every subject published by the server
    is tracked. Linear and angular velocities use frame-synchronous
    derivatives followed by a one-pole low-pass filter.
    """

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        *,
        subject_names: Sequence[str] | None = None,
        derivative_cutoff_hz: float = DEFAULT_DERIVATIVE_CUTOFF_HZ,
    ) -> None:
        self.host = host
        self.subject_names = (
            None if subject_names is None else tuple(subject_names)
        )
        self.derivative_cutoff_hz = derivative_cutoff_hz
        self._client: ViconDataStream.Client | None = None
        self._segments: dict[str, str] = {}
        self._motion: dict[str, _MotionHistory] = {}

    def open(self) -> "ViconTracker":
        """Connect to Vicon and resolve the root segment of each subject."""

        client = ViconDataStream.Client()
        client.Connect(self.host)
        client.SetBufferSize(1)
        client.EnableSegmentData()
        client.SetStreamMode(
            ViconDataStream.Client.StreamMode.EServerPush
        )
        client.SetAxisMapping(
            ViconDataStream.Client.AxisMapping.EForward,
            ViconDataStream.Client.AxisMapping.ELeft,
            ViconDataStream.Client.AxisMapping.EUp,
        )
        client.GetFrame()

        available = tuple(str(name) for name in client.GetSubjectNames())
        names = available if self.subject_names is None else self.subject_names
        self._segments = {
            name: str(client.GetSubjectRootSegmentName(name))
            for name in names
        }
        self._motion.clear()
        self._client = client
        return self

    def read(self) -> ViconFrame:
        """Read the next frame and return every visible rigid-body state."""

        if self._client is None:
            raise RuntimeError("ViconTracker is not open.")

        client = self._client
        client.GetFrame()
        receive_time_s = time.perf_counter()
        frame_number = int(client.GetFrameNumber())
        frame_rate_hz = float(client.GetFrameRate())
        latency_s = float(client.GetLatencyTotal())
        states: dict[str, RigidBodyState] = {}
        occluded: list[str] = []

        for subject_name, segment_name in self._segments.items():
            pose = _read_pose(client, subject_name, segment_name)
            if pose is None:
                occluded.append(subject_name)
                self._motion.pop(subject_name, None)
                continue

            position_m, euler_xyz_rad, quaternion_xyzw, rotation = pose
            states[subject_name] = self._state_from_pose(
                subject_name,
                frame_number,
                frame_rate_hz,
                position_m,
                euler_xyz_rad,
                quaternion_xyzw,
                rotation,
            )

        return ViconFrame(
            frame_number=frame_number,
            estimated_capture_time_s=receive_time_s - latency_s,
            frame_rate_hz=frame_rate_hz,
            latency_s=latency_s,
            states=states,
            occluded_subjects=tuple(occluded),
        )

    def close(self) -> None:
        """Disconnect from the Vicon DataStream server."""

        if self._client is not None:
            self._client.Disconnect()
            self._client = None

    def __enter__(self) -> "ViconTracker":
        return self.open()

    def __exit__(self, *_: object) -> None:
        self.close()

    def _state_from_pose(
        self,
        subject_name: str,
        frame_number: int,
        frame_rate_hz: float,
        position_m: _FloatArray,
        euler_xyz_rad: _FloatArray,
        quaternion_xyzw: _FloatArray,
        rotation: _FloatArray,
    ) -> RigidBodyState:
        previous = self._motion.get(subject_name)
        velocity_world_m_s: _FloatArray = np.zeros(3, dtype=np.float64)
        angular_velocity_world_rad_s: _FloatArray = np.zeros(
            3,
            dtype=np.float64,
        )
        motion_valid = (
            previous is not None
            and frame_number > previous.frame_number
            and frame_rate_hz > 0.0
        )

        if motion_valid and previous is not None:
            frame_count = frame_number - previous.frame_number
            dt_s = frame_count / frame_rate_hz
            raw_velocity = (
                position_m - previous.position_m
            ) / dt_s
            relative_world_rotation = (
                rotation @ previous.rotation_body_to_world.T
            )
            raw_angular_velocity_world = _rotation_vector(
                relative_world_rotation
            ) / dt_s
            if self.derivative_cutoff_hz > 0.0:
                alpha = 1.0 - np.exp(
                    -2.0
                    * np.pi
                    * self.derivative_cutoff_hz
                    * dt_s
                )
                velocity_world_m_s = (
                    previous.velocity_world_m_s
                    + alpha
                    * (
                        raw_velocity
                        - previous.velocity_world_m_s
                    )
                )
                angular_velocity_world_rad_s = (
                    previous.angular_velocity_world_rad_s
                    + alpha
                    * (
                        raw_angular_velocity_world
                        - previous.angular_velocity_world_rad_s
                    )
                )
            else:
                velocity_world_m_s = raw_velocity
                angular_velocity_world_rad_s = (
                    raw_angular_velocity_world
                )

        self._motion[subject_name] = _MotionHistory(
            frame_number=frame_number,
            position_m=position_m,
            rotation_body_to_world=rotation,
            velocity_world_m_s=velocity_world_m_s,
            angular_velocity_world_rad_s=angular_velocity_world_rad_s,
        )
        velocity_body_m_s = rotation.T @ velocity_world_m_s
        angular_velocity_body_rad_s = (
            rotation.T @ angular_velocity_world_rad_s
        )
        return RigidBodyState(
            position_m=_tuple3(position_m),
            euler_xyz_rad=_tuple3(euler_xyz_rad),
            quaternion_xyzw=_tuple4(quaternion_xyzw),
            velocity_world_m_s=_tuple3(velocity_world_m_s),
            velocity_body_m_s=_tuple3(velocity_body_m_s),
            angular_velocity_body_rad_s=_tuple3(
                angular_velocity_body_rad_s
            ),
            motion_valid=motion_valid,
        )


def _read_pose(
    client: ViconDataStream.Client,
    subject_name: str,
    segment_name: str,
) -> tuple[_FloatArray, _FloatArray, _FloatArray, _FloatArray] | None:
    translation, translation_occluded = (
        client.GetSegmentGlobalTranslation(subject_name, segment_name)
    )
    euler, euler_occluded = client.GetSegmentGlobalRotationEulerXYZ(
        subject_name,
        segment_name,
    )
    quaternion, quaternion_occluded = (
        client.GetSegmentGlobalRotationQuaternion(
            subject_name,
            segment_name,
        )
    )
    if (
        translation_occluded
        or euler_occluded
        or quaternion_occluded
    ):
        return None

    position_m = np.asarray(translation, dtype=float) / 1000.0
    euler_xyz_rad = np.asarray(euler, dtype=float)
    quaternion_xyzw = np.asarray(quaternion, dtype=float)
    rotation = _quaternion_to_rotation(quaternion_xyzw)
    return position_m, euler_xyz_rad, quaternion_xyzw, rotation


def _quaternion_to_rotation(
    quaternion_xyzw: _FloatArray,
) -> _FloatArray:
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    quaternion = quaternion / np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _rotation_vector(rotation: _FloatArray) -> _FloatArray:
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    vee = np.asarray(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=np.float64,
    )
    if angle < 1e-6:
        return 0.5 * vee
    return angle * vee / (2.0 * np.sin(angle))


def _tuple3(values: _FloatArray) -> _Vector3:
    return float(values[0]), float(values[1]), float(values[2])


def _tuple4(values: _FloatArray) -> _Quaternion:
    return (
        float(values[0]),
        float(values[1]),
        float(values[2]),
        float(values[3]),
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream full states from the Imperial flight arena."
    )
    parser.add_argument(
        "subjects",
        nargs="*",
        help="Vicon subject names; omit to track every subject.",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Vicon server address (default: {DEFAULT_HOST}).",
    )
    return parser.parse_args()


def _run_cli() -> None:
    args = _parse_args()
    subject_names = tuple(args.subjects) or None
    try:
        with ViconTracker(
            host=args.host,
            subject_names=subject_names,
        ) as tracker:
            while True:
                frame = tracker.read()
                if frame.frame_number % 20:
                    continue
                print(
                    f"frame={frame.frame_number} "
                    f"rate={frame.frame_rate_hz:.1f} Hz "
                    f"latency={1000.0 * frame.latency_s:.1f} ms"
                )
                for subject_name, state in frame.states.items():
                    print(f"  {subject_name}: {state}")
                if frame.occluded_subjects:
                    print(
                        "  occluded: "
                        + ", ".join(frame.occluded_subjects)
                    )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    _run_cli()
