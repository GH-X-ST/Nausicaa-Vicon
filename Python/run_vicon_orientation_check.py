"""Check an aircraft rigid body's Vicon pose and motion directions."""

import argparse
import time
from typing import TypeAlias

import numpy as np

from vicon_tracker import DEFAULT_HOST, RigidBodyState, ViconTracker


_REFERENCE_SAMPLE_COUNT = 50
_DEFAULT_MOTION_DURATION_S = 3.0
_MIN_TRANSLATION_M = 0.15
_MIN_ROTATION_RAD = np.deg2rad(8.0)
_MIN_ANGULAR_RATE_RAD_S = 0.10
_Check: TypeAlias = tuple[str, str, str, int, int]
_CHECKS: tuple[_Check, ...] = (
    (
        "forward",
        "move the aircraft forward along +X and hold",
        "translation",
        0,
        1,
    ),
    ("left", "move the aircraft left along +Y and hold", "translation", 1, 1),
    ("up", "move the aircraft upward along +Z and hold", "translation", 2, 1),
    (
        "roll right",
        "roll right (right wing down) and hold",
        "rotation",
        0,
        1,
    ),
    (
        "pitch up",
        "pitch the nose up and hold",
        "rotation",
        1,
        -1,
    ),
    (
        "yaw right",
        "yaw the nose right and hold",
        "rotation",
        2,
        -1,
    ),
)


def run_vicon_orientation_check(
    subject_name: str,
    *,
    host: str = DEFAULT_HOST,
    motion_duration_s: float = _DEFAULT_MOTION_DURATION_S,
) -> bool:
    """Guide an aircraft pose and motion sign check."""

    with ViconTracker(
        host=host,
        subject_names=(subject_name,),
    ) as tracker:
        print(
            "Align the aircraft nose with +X, left wing with +Y, "
            "and top with +Z."
        )
        input("Hold the reference pose, then press Enter.")
        reference = _collect_samples(
            tracker,
            subject_name,
            sample_count=_REFERENCE_SAMPLE_COUNT,
        )
        reference_position = np.mean(
            [state.position_m for state in reference],
            axis=0,
        )
        reference_euler = _mean_euler(reference)

        passed = True
        for name, instruction, kind, axis, expected_sign in _CHECKS:
            input(
                "\nReturn to the reference pose. Press Enter, "
                f"then {instruction}."
            )
            samples = _capture_motion(
                tracker,
                subject_name,
                motion_duration_s,
            )
            if not samples:
                print(f"FAIL: {name}: aircraft not visible")
                passed = False
                continue
            pose_change, peak_rate = _measure_change(
                samples,
                reference_position,
                reference_euler,
                kind,
                axis,
            )
            if kind == "rotation":
                step_passed = (
                    expected_sign * pose_change >= _MIN_ROTATION_RAD
                    and peak_rate is not None
                    and expected_sign * peak_rate >= _MIN_ANGULAR_RATE_RAD_S
                )
            else:
                step_passed = expected_sign * pose_change >= _MIN_TRANSLATION_M
            passed = passed and step_passed
            status = "PASS" if step_passed else "FAIL"
            if kind == "rotation":
                angle_deg = float(np.rad2deg(pose_change))
                rate_deg_s = float(np.rad2deg(peak_rate or 0.0))
                print(
                    f"{status}: {name}: change={angle_deg:+.3f} deg, "
                    f"peak_rate={rate_deg_s:+.3f} deg/s"
                )
            else:
                print(f"{status}: {name}: change={pose_change:+.3f} m")

    print(f"\nOverall: {'PASS' if passed else 'FAIL'}")
    return passed


def _collect_samples(
    tracker: ViconTracker,
    subject_name: str,
    *,
    sample_count: int,
) -> list[RigidBodyState]:
    samples: list[RigidBodyState] = []
    while len(samples) < sample_count:
        state = tracker.read().states.get(subject_name)
        if state is not None:
            samples.append(state)
    return samples


def _capture_motion(
    tracker: ViconTracker,
    subject_name: str,
    duration_s: float,
) -> list[RigidBodyState]:
    samples: list[RigidBodyState] = []
    start_time_s = time.perf_counter()
    while time.perf_counter() - start_time_s < duration_s:
        state = tracker.read().states.get(subject_name)
        if state is not None:
            samples.append(state)
    return samples


def _mean_euler(samples: list[RigidBodyState]) -> np.ndarray:
    values = np.asarray(
        [state.euler_xyz_rad for state in samples],
        dtype=float,
    )
    return np.arctan2(
        np.mean(np.sin(values), axis=0),
        np.mean(np.cos(values), axis=0),
    )


def _measure_change(
    samples: list[RigidBodyState],
    reference_position: np.ndarray,
    reference_euler: np.ndarray,
    kind: str,
    axis: int,
) -> tuple[float, float | None]:
    final_samples = samples[-max(1, len(samples) // 5) :]
    if kind == "rotation":
        final_angle = _mean_euler(final_samples)[axis]
        pose_change = _wrap_angle(final_angle - reference_euler[axis])
        motion = [
            state.angular_velocity_body_rad_s[axis]
            for state in samples
            if state.motion_valid
        ]
        peak_rate = max(motion, key=abs) if motion else None
    else:
        final_position = np.mean(
            [state.position_m for state in final_samples],
            axis=0,
        )
        pose_change = final_position[axis] - reference_position[axis]
        peak_rate = None
    return float(pose_change), peak_rate


def _wrap_angle(angle_rad: float) -> float:
    return float((angle_rad + np.pi) % (2.0 * np.pi) - np.pi)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check an aircraft rigid body's Vicon axis directions."
    )
    parser.add_argument(
        "subject",
        help="Aircraft subject name in Vicon Tracker.",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Vicon server address (default: {DEFAULT_HOST}).",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=_DEFAULT_MOTION_DURATION_S,
        help="Time allowed for each motion.",
    )
    return parser.parse_args()


def main() -> None:
    """Run the command-line orientation check."""

    args = _parse_args()
    passed = run_vicon_orientation_check(
        args.subject,
        host=args.host,
        motion_duration_s=args.duration_s,
    )
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
