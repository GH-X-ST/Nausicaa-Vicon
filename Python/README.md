# Imperial Flight Arena Vicon Client

This folder contains the minimal Python interface for reading multiple rigid
bodies from the Imperial College London flight arena. One Vicon frame is read
at a time, so all returned objects are synchronized.

## Returned state

`ViconTracker.read()` returns a `ViconFrame` with:

- Vicon frame number and frame rate
- estimated capture time on the local monotonic clock
- total Vicon latency
- a `states` mapping for every subject whose root pose is not occluded
- an `occluded_subjects` tuple for subjects whose root pose is occluded

Each `RigidBodyState` contains:

- global position in metres
- global Euler XYZ angles in radians
- global quaternion in `(x, y, z, w)` order
- linear velocity in the global frame, in metres per second
- linear velocity in the rigid body's frame, in metres per second
- angular velocity in the rigid body's frame, in radians per second
- `motion_valid`, which is true when velocities use two advancing visible poses

The DataStream axes are configured as X forward, Y left, and Z up. The body
frame is the subject's root-segment frame defined in Vicon Tracker. Velocities
are derived from successive visible poses using Vicon frame timing, then
filtered with an 8 Hz one-pole low-pass filter. They are zero on the first
visible frame after startup or occlusion because no previous visible pose is
available. `motion_valid` is also false if a frame number does not advance.

The arena normally streams at 200 Hz; use `frame_rate_hz` from each returned
frame as the actual rate.

Positions and orientations remain in the Vicon global frame. No
object-specific origin, attitude correction, or arena-layout transform is
applied.

`estimated_capture_time_s` is the local `time.perf_counter()` value at frame
receipt minus Vicon's reported processing latency. It is suitable for timing
within the running process, not as a wall-clock or server-synchronized
timestamp.

## Before running

1. Use a 64-bit Windows computer connected to the flight-arena Vicon network.
2. Start Vicon Tracker and enable its DataStream output.
3. Create or load each rigid body in Vicon Tracker. Give every object a unique
   subject name and confirm that it is visible.
4. Install 64-bit Python 3.12.
5. Install the Microsoft Visual C++ 2015-2022 x64 Redistributable if it is not
   already present.

Subject names are case-sensitive. The arena server address is
`192.168.0.100:801`.

## Install

Open PowerShell in this folder and run:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

The required Windows Python 3 files from the Vicon DataStream SDK are included
in `vicon_dssdk`. The bundled runtime reports version 1.12.0, and its license
is retained in `vicon_dssdk/LICENSE`.

## Run

Track every subject currently published by Vicon:

```powershell
.\.venv\Scripts\python.exe .\vicon_tracker.py
```

Track selected subjects:

```powershell
.\.venv\Scripts\python.exe .\vicon_tracker.py "Object A" "Object B"
```

The command prints one complete sample every 20 Vicon frames. Press `Ctrl+C`
to stop.

## Check an aircraft rigid body

Define the aircraft root segment with X toward the nose, Y toward the left
wing, and Z upward. Then check its position, attitude, and angular-velocity
directions:

```powershell
.\.venv\Scripts\python.exe .\run_vicon_orientation_check.py "Aircraft"
```

The check guides six movements. Forward, left, and upward motion must be
positive along X, Y, and Z. With the right-handed X-forward, Y-left, Z-up
frame, right roll is positive about X, while nose-up pitch and nose-right yaw
are negative about Y and Z. Use movements of at least 0.15 m or 8 degrees;
rotations must exceed 0.10 rad/s.

To compare a stationary aircraft with a known arena pose, run:

```powershell
.\.venv\Scripts\python.exe .\run_vicon_frame_calibration.py "Aircraft" `
    --known-position-m 0 0 0 `
    --known-euler-deg 0 0 0
```

Replace the known position and Euler XYZ angles with the physical reference
pose, using the same axis directions as the Vicon stream. The command reports
the mean pose and the wrapped measured-minus-reference error. This is a
single-pose comparison, not a coordinate-frame transform. It does not change
the Vicon configuration, write a calibration file, or transform the states
returned by `ViconTracker`. A reported error can come from either the Vicon
arena frame or the selected aircraft's root-segment definition.

Each check uses one aircraft at a time. Repeat it for another aircraft when
needed; normal tracking remains synchronized across all requested subjects.

## Use from another script

```python
from vicon_tracker import ViconTracker


with ViconTracker(subject_names=("Object A", "Object B")) as tracker:
    while True:
        frame = tracker.read()
        for subject_name, state in frame.states.items():
            print(subject_name, state.position_m, state.velocity_body_m_s)
```

Omit `subject_names` to track all subjects available when `open()` runs.
Reopen the tracker after adding or renaming a subject. A temporarily occluded
subject is absent from `frame.states` and listed in
`frame.occluded_subjects`.

If the SDK reports `ClientConnectionFailed`, check the network connection,
server address, and DataStream output. If it reports `InvalidSubjectName`,
compare the requested name with the case-sensitive name in Vicon Tracker.
