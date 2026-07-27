<p align="center">
  <a href="https://github.com/GH-X-ST/Nausicaa">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/Workflow_Repository-Nausicaa-ffff00?style=for-the-badge&labelColor=0d1117">
      <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/Workflow_Repository-Nausicaa-0000cd?style=for-the-badge&labelColor=ffffff">
      <img src="https://img.shields.io/badge/Workflow-Nausicaa-7b1fa2?style=for-the-badge&labelColor=ffffff" alt="Nausicaa workflow repository">
    </picture>
  </a>
</p>

<p align="center">
  <sub>General-purpose Vicon DataStream clients for synchronised multi-object rigid-body tracking</sub><br>
  <sub>in the Imperial College London flight arena</sub><br>
</p>

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
    <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-ffffff?style=for-the-badge&labelColor=0d1117">
    <source media="(prefers-color-scheme: light)" srcset="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-232333?style=for-the-badge&labelColor=ffffff">
    <img src="https://img.shields.io/badge/Vicon_DataStream_SDK-1.12.0-6e7781?style=for-the-badge&labelColor=ffffff" alt="Vicon DataStream SDK 1.12.0">
  </picture>
</p>

<br>

## About

Nausicaa-Vicon provides compact Python and MATLAB interfaces for the Vicon system in the Imperial College London flight arena. Both clients acquire one server frame at a time, keeping every requested rigid body synchronised while returning its global pose and body-frame motion state. Subjects can be discovered automatically or selected by their case-sensitive names.

The implementations follow their host languages rather than mechanically translating one into the other. Python returns a rich frame object with timing, latency, occlusion, quaternion, and global-velocity information. MATLAB returns the conventional 12-state matrix `[x, y, z, roll, pitch, yaw, u, v, w, p, q, r]` with row validity and frame number. Both folders also provide corresponding aircraft orientation and stationary reference-pose checks.

<p align="center">
  <img src="https://raw.githubusercontent.com/GH-X-ST/Nausicaa/main/A_Miscellaneous/A_Readme/3.2.2.jpg" alt="Selected flight test sensing, computation, and command architecture" width="100%"><br>
  <sup><em>Selected flight test sensing, computation, and command architecture in <a href="https://github.com/GH-X-ST/Nausicaa">GH-X-ST/Nausicaa</a>.</em></sup>
</p>

The diagram gives the wider experimental context. This repository isolates only the reusable Vicon sensing interface and its physical checks; it does not include the flight controller, radio-command path, fan configuration, arena layout, or other experiment-specific code.

---

## Interfaces

| Interface | Returned data | Entry point |
|---|---|---|
| Python | Frame number and rate, capture-time estimate, latency, visible and occluded subjects, position, Euler XYZ attitude, quaternion XYZW, global and body linear velocity, body angular velocity, and motion validity; derivatives use a configurable one-pole low-pass filter with an 8 Hz default | [`Python/vicon_tracker.py`](Python/vicon_tracker.py) |
| MATLAB | One synchronised N-by-12 state matrix with unfiltered finite-difference body velocities, validity by subject, and frame number | [`MATLAB/viconTracker.m`](MATLAB/viconTracker.m) |

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
2. Start Vicon Tracker, enable DataStream output, and confirm that each rigid body is visible under a unique subject name.
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

In a fresh MATLAB session opened from the repository root, read once to initialise the finite-difference motion estimate before acquiring the complete state:

```matlab
cd MATLAB
dotnetenv("framework")
tracker = viconTracker(["Object A", "Object B"]);
tracker.read();
[states, valid, frameNumber] = tracker.read();
delete(tracker);
```

Call `viconTracker` without subject names to track every subject available when the connection opens.

### Check an aircraft rigid body

Run the guided orientation check:

```powershell
.\Python\.venv\Scripts\python.exe .\Python\run_vicon_orientation_check.py "Aircraft"
```

```matlab
passed = runViconOrientationCheck("Aircraft");
```

The stationary reference-pose comparisons report the difference between a measured mean pose and a known arena pose without changing the Vicon configuration or transforming later tracking data; see the language-specific guides for their arguments. Each guided check operates on one rigid body at a time, while the normal clients track multiple bodies.

### SDK licences

The required Vicon runtime files are included with their accompanying licences in [`Python/vicon_dssdk/LICENSE`](Python/vicon_dssdk/LICENSE) and [`MATLAB/vicon_sdk/LICENSE`](MATLAB/vicon_sdk/LICENSE).
