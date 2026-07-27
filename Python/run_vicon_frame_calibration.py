"""Compare a Vicon aircraft pose with a known reference pose."""

import argparse
from dataclasses import dataclass
from typing import TypeAlias

import numpy as np

from vicon_tracker import DEFAULT_HOST, ViconTracker


_SAMPLE_COUNT = 200
_Vector3: TypeAlias = tuple[float, float, float]


@dataclass(frozen=True)
class PoseComparison:
    """Measured aircraft pose and its error from the reference pose."""

    position_mean_m: _Vector3
    position_error_m: _Vector3
    euler_xyz_mean_rad: _Vector3
    euler_xyz_error_rad: _Vector3


def run_vicon_frame_calibration(
    subject_name: str,
    known_position_m: _Vector3,
    known_euler_xyz_deg: _Vector3,
    *,
    host: str = DEFAULT_HOST,
    sample_count: int = _SAMPLE_COUNT,
) -> PoseComparison:
    """Compare a stationary aircraft with a known arena pose."""

    positions: list[_Vector3] = []
    euler_angles: list[_Vector3] = []
    with ViconTracker(
        host=host,
        subject_names=(subject_name,),
    ) as tracker:
        while len(positions) < sample_count:
            state = tracker.read().states.get(subject_name)
            if state is not None:
                positions.append(state.position_m)
                euler_angles.append(state.euler_xyz_rad)

    result = _summarize_pose(
        np.asarray(positions),
        np.asarray(euler_angles),
        np.asarray(known_position_m),
        np.deg2rad(known_euler_xyz_deg),
    )
    print(f"Position mean (m): {_format(result.position_mean_m)}")
    print(
        "Position error, measured-reference (m): "
        f"{_format(result.position_error_m)}"
    )
    print(
        "Euler XYZ mean (deg): "
        f"{_format(np.rad2deg(result.euler_xyz_mean_rad))}"
    )
    print(
        "Euler XYZ error, measured-reference (deg): "
        f"{_format(np.rad2deg(result.euler_xyz_error_rad))}"
    )
    return result


def _summarize_pose(
    positions: np.ndarray,
    euler_angles: np.ndarray,
    known_position: np.ndarray,
    known_euler: np.ndarray,
) -> PoseComparison:
    position_mean = np.mean(positions, axis=0)
    euler_mean = np.arctan2(
        np.mean(np.sin(euler_angles), axis=0),
        np.mean(np.cos(euler_angles), axis=0),
    )
    return PoseComparison(
        position_mean_m=_tuple3(position_mean),
        position_error_m=_tuple3(position_mean - known_position),
        euler_xyz_mean_rad=_tuple3(euler_mean),
        euler_xyz_error_rad=_tuple3(_wrap_angles(euler_mean - known_euler)),
    )


def _wrap_angles(angles_rad: np.ndarray) -> np.ndarray:
    return (angles_rad + np.pi) % (2.0 * np.pi) - np.pi


def _tuple3(values: np.ndarray) -> _Vector3:
    return float(values[0]), float(values[1]), float(values[2])


def _format(values: _Vector3 | np.ndarray) -> str:
    return np.array2string(
        np.asarray(values),
        precision=4,
        floatmode="fixed",
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare a Vicon aircraft pose with a known reference."
    )
    parser.add_argument(
        "subject",
        help="Aircraft subject name in Vicon Tracker.",
    )
    parser.add_argument(
        "--known-position-m",
        type=float,
        nargs=3,
        required=True,
        help="Known aircraft position as X Y Z in metres.",
    )
    parser.add_argument(
        "--known-euler-deg",
        type=float,
        nargs=3,
        required=True,
        help="Known aircraft Euler XYZ angles in degrees.",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Vicon server address (default: {DEFAULT_HOST}).",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=_SAMPLE_COUNT,
        help=f"Visible frames to average (default: {_SAMPLE_COUNT}).",
    )
    return parser.parse_args()


def main() -> None:
    """Run the command-line pose comparison."""

    args = _parse_args()
    print("Hold the aircraft still at the specified reference pose.")
    input("Press Enter to collect samples.")
    run_vicon_frame_calibration(
        args.subject,
        tuple(args.known_position_m),
        tuple(args.known_euler_deg),
        host=args.host,
        sample_count=args.samples,
    )


if __name__ == "__main__":
    main()
