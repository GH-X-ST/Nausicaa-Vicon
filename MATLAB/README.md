# Imperial Flight Arena Vicon Client

This folder provides a MATLAB interface for reading synchronized full states
from multiple rigid bodies in the Imperial College London flight arena. Its
frame contents and motion logic match the Python interface.

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

Display every subject available when the connection opens:

```matlab
runViconTracker()
```

Display selected subjects:

```matlab
runViconTracker(["Object A", "Object B"])
```

Use the tracker from another program:

```matlab
tracker = viconTracker(["Object A", "Object B"]);
frameData = tracker.read();
state = frameData.states("Object A");
delete(tracker);
```

Use another server address or derivative cutoff:

```matlab
tracker = viconTracker( ...
    "Object A", ...
    Host="192.168.0.100:801", ...
    DerivativeCutoffHz=8.0);
```

Omit the subject names to track every subject available when the connection
opens. Set `DerivativeCutoffHz=0` to return unfiltered derivatives. Disconnect
with `delete(tracker)` when finished. `runViconTracker()` disconnects when
`Ctrl+C` stops the function.

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

Hold a stationary aircraft at a known pose in the arena frame, then run:

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

## Returned frame

`viconTracker.read()` returns one structure with:

- `frameNumber`
- `estimatedCaptureTimeS`
- `frameRateHz`
- `latencyS`
- `states`, a dictionary keyed by visible subject name
- `occludedSubjects`, a string array of occluded subject names

Each value in `states` contains:

- `positionM`, global position in metres
- `eulerXYZRad`, global Euler XYZ angles in radians
- `quaternionXYZW`, global quaternion in `(x, y, z, w)` order
- `velocityWorldMPerS`, global linear velocity in metres per second
- `velocityBodyMPerS`, body-frame linear velocity in metres per second
- `angularVelocityBodyRadPerS`, body-frame angular velocity in radians per
  second
- `motionValid`, true when the motion estimate uses two advancing visible
  poses

The configured axes are X forward, Y left, and Z up. The body frame is the
subject's root-segment frame in Vicon Tracker. Every call to `read` acquires
one frame, so all returned subjects are synchronized.

Velocities are derived from successive visible poses using Vicon frame timing
and filtered with a configurable one-pole low-pass filter whose default cutoff
is 8 Hz. They are zero and `motionValid` is false on the first visible frame
after connection or occlusion, or when the frame number does not advance.
An occluded subject is absent from `states`, listed in `occludedSubjects`, and
has its motion history cleared.

Positions and orientations remain in the Vicon global frame. No
object-specific origin, attitude correction, or arena-layout transform is
applied.

`estimatedCaptureTimeS` is the local high-resolution monotonic timer value at
frame receipt minus Vicon's reported processing latency. It uses the same
Windows performance-counter timebase as Python `time.perf_counter()` and is
not a wall-clock or server-synchronized timestamp.
