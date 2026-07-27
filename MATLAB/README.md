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
