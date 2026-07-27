# Imperial Flight Arena Vicon Client

This folder provides a MATLAB interface for reading synchronized 12-state
vectors from multiple rigid bodies in the Imperial College London flight
arena.

## Before running

1. Use 64-bit MATLAB on a Windows computer connected to the flight-arena
   Vicon network.
2. Start Vicon Tracker and enable DataStream output.
3. Create or load each rigid body in Vicon Tracker. Give every object a unique
   subject name and confirm that it is visible.
4. Install the Microsoft Visual C++ 2015-2022 x64 Redistributable.
5. Open MATLAB in this folder or add this folder to the MATLAB path.

The included Vicon SDK requires .NET Framework. In a fresh MATLAB session,
select that runtime before loading any .NET assembly:

```matlab
dotnetenv("framework")
```

Subject names are case-sensitive. The default arena server address is
`192.168.0.100:801`. The required Vicon runtime is included in `vicon_sdk`.

## Track rigid bodies

Track every subject available when the connection opens:

```matlab
tracker = viconTracker;
[states, valid, frameNumber] = tracker.read();
```

Track selected subjects:

```matlab
tracker = viconTracker(["Object A", "Object B"]);
[states, valid, frameNumber] = tracker.read();
```

Use another server address:

```matlab
tracker = viconTracker("Object A", Host="192.168.0.100:801");
```

Disconnect when finished:

```matlab
delete(tracker);
```

## Check an aircraft rigid body

Run the orientation check once for each aircraft subject:

```matlab
passed = runViconOrientationCheck("Aircraft");
```

At the reference pose, align the nose with +X, the left wing with +Y, and
the top of the aircraft with +Z. The check then guides six motions. Forward,
left, and up must increase X, Y, and Z. Right roll must increase roll and
`p`; nose-up pitch must decrease pitch and `q`; right yaw must decrease yaw
and `r`. A translation passes at 0.15 m. A rotation passes at 8 degrees and
0.10 rad/s.

Use another server or change the time allowed for each motion with
name-value arguments:

```matlab
passed = runViconOrientationCheck( ...
    "Aircraft", ...
    Host="192.168.0.100:801", ...
    MotionDurationSeconds=4.0);
```

Compare a stationary aircraft with a known pose in the arena frame:

```matlab
knownPositionM = [1.0, 0.0, 0.5];
knownEulerXYZDeg = [0.0, 0.0, 90.0];
result = runViconFrameCalibration( ...
    "Aircraft", ...
    knownPositionM, ...
    knownEulerXYZDeg);
```

`knownPositionM` is `[X Y Z]` in metres and `knownEulerXYZDeg` is
`[roll pitch yaw]` in degrees, both in the configured global Vicon frame.
The returned errors are measured minus reference. This is a single-pose
comparison, not a coordinate-frame transform; it does not change the Vicon
configuration or subsequent tracking data. Use `SampleCount=200` to set the
number of visible frames to average.

These checks select one aircraft because the user moves it by hand. Normal
tracking remains available for multiple synchronized rigid bodies through
`viconTracker`.

## Returned state

Each row of `states` represents the subject at the same row of
`tracker.SubjectNames`. The columns are listed by `tracker.StateNames`:

```text
[x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
```

- `x`, `y`, and `z` are global positions in metres.
- `roll`, `pitch`, and `yaw` are global Euler XYZ angles in radians.
- `u`, `v`, and `w` are body-frame linear velocities in metres per second.
- `p`, `q`, and `r` are body-frame angular velocities in radians per second.

The configured axes are X forward, Y left, and Z up. The body frame is the
subject's root-segment frame in Vicon Tracker. Every call to `read` acquires
one frame, so all rows are synchronized.

`valid` is true when all 12 values in that row are available. Velocity and
angular velocity require two advancing, visible frames, so `valid` is false
and the unavailable values are `NaN` on the first frame after connection or
occlusion. An occluded subject keeps its row, with all values set to `NaN`.
