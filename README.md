<p align="center">
  <a href="https://gh-x-st.github.io/Nausicaa-Thesis/">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/Read-Thesis-ffffff?style=for-the-badge&labelColor=0d1117">
      <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/Read-Thesis-000000?style=for-the-badge&labelColor=ffffff">
      <img src="https://img.shields.io/badge/Read-Thesis-6e7781?style=for-the-badge&labelColor=ffffff" alt="Read thesis">
    </picture>
  </a>
</p>

<p align="center">
  <sub>Vicon DataStream clients for synchronised multi-object rigid-body tracking</sub><br>
  <sub>in the Imperial College London flight arena</sub><br>
</p>

![Vicon system cover light](Vicon.png#gh-light-mode-only)
![Vicon system cover dark](Vicon_Dark.png#gh-dark-mode-only)

<br>

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/Python-3.12-FFE873?style=for-the-badge&labelColor=0d1117">
    <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/Python-3.12-306998?style=for-the-badge&labelColor=ffffff">
    <img src="https://img.shields.io/badge/Python-3.12-3776ab?style=for-the-badge&labelColor=ffffff" alt="Python 3.12">
  </picture>
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/MATLAB-R2026a-fd8000?style=for-the-badge&labelColor=0d1117">
    <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/MATLAB-R2026a-006da8?style=for-the-badge&labelColor=ffffff">
    <img src="https://img.shields.io/badge/MATLAB-R2026a-7b1fa2?style=for-the-badge&labelColor=ffffff" alt="MATLAB R2026a">
  </picture>
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-fa8072?style=for-the-badge&labelColor=0d1117">
    <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-008080?style=for-the-badge&labelColor=ffffff">
    <img src="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-6e7781?style=for-the-badge&labelColor=ffffff" alt="Vicon DataStream SDK 1.12.0">
  </picture>
</p>

<br>

## About

Nausicaa-Vicon provides compact Python and MATLAB interfaces for the Vicon system in the Imperial College London flight arena. Both clients acquire one server frame at a time, keeping every requested rigid body synchronised while returning the same pose, motion, timing, latency, visibility, and frame information.

The implementations use language-native containers while following the same logic. Each visible state contains global position, Euler and quaternion attitude, global and body linear velocity, body angular velocity, and motion validity. Both folders also provide corresponding aircraft orientation and stationary reference-pose checks.

<p align="center">
  <img src="https://raw.githubusercontent.com/GH-X-ST/Nausicaa/main/A_Miscellaneous/A_Readme/3.2.2.jpg" alt="Selected flight test sensing, computation, and command architecture" width="100%"><br>
  <sup><em>Selected flight test sensing, computation, and command architecture.</em></sup>
</p>

---

## Capabilities and Sources

| Capability | Provider | Python source | MATLAB source |
|---|---|---|---|
| Synchronized frames, frame number and rate, and latency | Vicon DataStream SDK | [`ViconTracker.read()`](Python/vicon_tracker.py) | [`viconTracker.read()`](MATLAB/viconTracker.m) |
| Global position, Euler attitude, quaternion, and occlusion | Vicon DataStream SDK | [`_read_pose()`](Python/vicon_tracker.py) | [`readPose()`](MATLAB/viconTracker.m) |
| Multi-object frame packaging and unit conversion | This repository | [`ViconFrame` and `RigidBodyState`](Python/vicon_tracker.py) | [Frame structure and state dictionary](MATLAB/viconTracker.m) |
| Derived velocities, filtering, and body-frame transformation | This repository | [`_state_from_pose()`](Python/vicon_tracker.py) | [`stateFromPose()`](MATLAB/viconTracker.m) |
| Continuous state display | This repository | [`vicon_tracker.py`](Python/vicon_tracker.py) | [`runViconTracker.m`](MATLAB/runViconTracker.m) |
| Orientation and reference-pose checks | This repository | [Python checks](Python/README.md#check-an-aircraft-rigid-body) | [MATLAB checks](MATLAB/README.md#check-an-aircraft-rigid-body) |

Both implementations return the same information and apply the same processing logic. Python represents a frame with the `ViconFrame` and `RigidBodyState` dataclasses, while MATLAB uses a scalar frame structure.

Positions use metres, angles use radians, linear velocities use metres per second, and angular velocities use radians per second. The configured axes are X forward, Y left, and Z up. Position and attitude remain in the Vicon global frame; body velocities use the subject root-segment frame. No object-specific correction or arena transform is applied.

The folder guides describe each return type, occlusion behaviour, motion estimate, and check in detail:

- [Python interface and checks](Python/README.md)
- [MATLAB interface and checks](MATLAB/README.md)

---

## Setup and Use

### Reference environment

| Tool | Version |
|---|---|
| Windows | 11 25H2, 64-bit |
| Python | 3.12.11 |
| MATLAB | R2026a |
| Vicon Tracker | 3.9 |
| Vicon DataStream SDK | 1.12.0 |

Before running either client:

1. Connect the computer to the flight-arena Vicon network.
2. Start Vicon Tracker, enable DataStream output, and ensure each object is visible under a unique subject name.
3. Install the Microsoft Visual C++ 2015-2022 x64 Redistributable.

The default server is `192.168.0.100:801` and can be changed in either interface.

### Clone

```powershell
git clone https://github.com/GH-X-ST/Nausicaa-Vicon.git
cd Nausicaa-Vicon
```

### Python

```powershell
py -3.12 -m venv .\Python\.venv
.\Python\.venv\Scripts\python.exe -m pip install -r .\Python\requirements.txt
.\Python\.venv\Scripts\python.exe .\Python\vicon_tracker.py "Object A" "Object B"
```

Omit the subject names to track every subject available when the connection opens.

### MATLAB

In a fresh MATLAB session opened from the repository root:

```matlab
cd MATLAB
dotnetenv("framework")
tracker = viconTracker(["Object A", "Object B"]);
frameData = tracker.read();
state = frameData.states("Object A");
delete(tracker);
```

Call `viconTracker` without subject names to track every subject available when the connection opens, or run `runViconTracker()` to display complete streaming states until `Ctrl+C`.

### Check an aircraft rigid body

Run the guided orientation check:

```powershell
.\Python\.venv\Scripts\python.exe .\Python\run_vicon_orientation_check.py "Aircraft"
```

```matlab
passed = runViconOrientationCheck("Aircraft");
```

The stationary reference-pose comparisons report the difference between a measured mean pose and a known arena pose without changing the Vicon configuration or transforming later tracking data. Each guided check operates on one rigid body at a time, while the normal clients track multiple bodies.
