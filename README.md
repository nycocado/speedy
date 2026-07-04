<div align="center">

![Speedy](media/photos/analogue-01.png)

# Speedy

A reactive autonomous racing vehicle combining vision-based lane following, LiDAR obstacle avoidance, and IMU/odometry heading-hold on a standalone Raspberry Pi 4 running ROS 2 Jazzy.

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-3da639.svg)](LICENSE)
![Status](https://img.shields.io/badge/status-completed-6f42c1)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp)
[![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![YOLO11n](https://img.shields.io/badge/YOLO11n-NCNN-00FFFF)](https://github.com/ultralytics/ultralytics)
[![Ansible](https://img.shields.io/badge/Ansible-EE0000?logo=ansible&logoColor=white)](https://www.ansible.com/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi%204-A22846?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)

[Portuguese](README.pt.md) | English

</div>

## About

**Speedy** is an autonomous racing vehicle built for reactive, high-performance track navigation. The project started as a hybrid architecture — a Raspberry Pi handling perception alongside an ESP32-S3 running micro-ROS for low-level actuation, bridged over XRCE-DDS — and was later collapsed into a **standalone Raspberry Pi 4** design where the hardware interface itself runs as a `ros2_control` plugin on the Pi. Moving the whole stack onto one board removed the serial round-trip between perception and actuation, so the vision pipeline and the control loop now share a deterministic timing budget.

Built for the Computer Engineering degree at **IADE — Universidade Europeia**, Speedy runs entirely on **ROS 2 Jazzy** and combines classical control with on-board deep learning: a PID lane-follower reacts to a real-time OpenCV line detector, a LiDAR-based local planner vetoes unsafe trajectories, and a YOLO11n model (exported to NCNN) spots obstacles and track signage.

## How it works

- **Vision-guided steering.** [`speedy_vision`](iot/software/src/speedy_vision) scans the camera frame for the two track lines, tracks them independently frame-to-frame (rather than collapsing them into one centroid), and publishes lateral and heading error. [`speedy_navigation`](iot/software/src/speedy_navigation)'s reactive controller turns that into a steering command with a PID loop plus heading feedforward, so it can anticipate curvature instead of just reacting to it.
- **Vision proposes, LiDAR vetoes.** Rather than fighting the vision output whenever an obstacle appears, the reactive controller simulates ~20 candidate steering arcs each cycle, discards any that a LiDAR scan says would collide, and picks the surviving arc closest to what the camera asked for (a lightweight Dynamic Window Approach). This lets the car thread narrow gaps and chicanes smoothly instead of over-correcting.
- **Blind heading-hold.** When both track lines are lost (e.g. mid-ramp), the controller falls back to a heading-hold driven by the EKF's absolute yaw, until the lines are reacquired.
- **Ramp handling.** A `FLOOR` detection from the obstacle model (bounding box above a minimum pixel size, debounced over a few frames) suppresses the now-unreliable line detector, holds the last confident straight-line yaw, and drives at a fixed momentum speed to carry the car over the ramp crest.
- **Obstacle and sign detection.** [`obstacle_detector_node`](iot/software/src/speedy_vision/speedy_vision/obstacle_detector_node.py) runs the YOLO11n/NCNN model in a dedicated process that always consumes the latest frame — stale frames are dropped rather than queued, so inference latency never compounds on the Pi 4.
- **State machine and safety.** [`speedy_supervisor`](iot/software/src/speedy_supervisor) arbitrates between manual (joystick) and autonomous `ros2_control` controllers via a gamepad button combo, and latches an E-stop that zeroes all actuator output until explicitly cleared.
- **Deterministic hardware interface.** [`speedy_control`](iot/software/src/speedy_control) is a C++ `ros2_control` hardware interface driving motor PWM and the BTS7960 driver, servo pulses via `pigpio`, and Hall-encoder odometry via `libgpiod` — kept off the Python/DDS path to avoid OS-induced jitter.
- **Remote telemetry.** A [Foxglove](https://foxglove.dev/) WebSocket bridge streams camera, LiDAR point clouds, and controller diagnostics live, for tuning gains and debugging without a monitor on the car.

## Architecture

### Hardware

| Component    | Part                                                  |
| ------------ | ----------------------------------------------------- |
| Compute      | Raspberry Pi 4 (4GB)                                  |
| LiDAR        | LDROBOT D500 / LD19 (ToF, 230400 baud)                |
| IMU          | MPU-6050 (onboard DMP)                                |
| Camera       | Raspberry Pi Camera OV5647 160°, 640×480              |
| Odometry     | 5× Hall-effect sensors (2 front wheels + motor shaft) |
| Drive motor  | JGB37-520 12V 600RPM, 2.3:1 gear ratio                |
| Motor driver | BTS7960 H-bridge                                      |
| Steering     | MG996R servo, Ackermann geometry, 0.251 m wheelbase   |
| Power        | LiPo 3S 11.1V 5000mAh                                 |

<table>
<tr>
<td width="50%"><img src="media/photos/studio-01.png" width="100%"><br>Compute deck — Pi 4 ports, LD19 LiDAR mount, ribbon cable to the camera</td>
<td width="50%"><img src="media/photos/studio-02.png" width="100%"><br>Motor driver heatsink and LiDAR mount, seen from the other side</td>
</tr>
</table>

### Software — ROS 2 workspace (`iot/software/src`)

| Package              | Responsibility                                                                                                                                                             |
| -------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `speedy_bringup`     | Launch files and per-subsystem config (camera, IMU, LiDAR, EKF, controllers, Foxglove)                                                                                     |
| `speedy_control`     | C++ `ros2_control` hardware interface (motor, servo, Hall encoders)                                                                                                        |
| `speedy_vision`      | Line detector (OpenCV) and obstacle/sign detector (YOLO11n NCNN)                                                                                                           |
| `speedy_navigation`  | Reactive controller: PID lane-following + LiDAR-vetoed steering arcs + heading-hold                                                                                        |
| `speedy_supervisor`  | Manual/autonomous state machine and E-stop                                                                                                                                 |
| `speedy_calibration` | Servo pulse → steering angle calibration                                                                                                                                   |
| `speedy_teleop`      | Joystick teleop, decoupled from whichever controller is currently active                                                                                                   |
| `speedy_dataset`     | On-track image collection for the YOLO dataset                                                                                                                             |
| `speedy_description` | URDF/xacro robot model, parametrized from `hardware.yaml`                                                                                                                  |
| `deps/`              | Vendored ROS 2 dependencies (`camera_ros`, `ldlidar_ros2`, `robot_localization`, `imu_tools`, `foxglove-sdk`, `rosx_introspection`, `mpu6050_driver`, …) as git submodules |

Perception and control are fully decoupled from provisioning: [`iot/ansible`](iot/ansible) is an Infrastructure-as-Code layer that provisions the Pi from a bare Raspberry Pi OS image (kernel overlays, network/hotspot, ROS 2, workspace build, systemd auto-start) and mirrors the same workspace into a Distrobox container for development on any Linux host.

## Perception and sensor fusion

- **Camera.** The OV5647 is calibrated (`camera_info.yaml`) with a checkerboard target; the resulting intrinsics/distortion coefficients feed the optional undistortion step in the dataset collector, and are exposed to any node via `camera_info`.
- **LiDAR.** The LD19 driver crops the raw scan to the forward 180° (`angle_crop_min/max = 90°/270°`), masking the rear half to NaN — this keeps scan-matching and the mini-DWA planner from reacting to obstacles behind the car and halves the points processed each cycle.
- **IMU.** The MPU6050 runs in a hybrid mode: orientation comes from the chip's onboard **DMP** (continuous gyro-bias correction, so yaw doesn't drift like raw-rate integration would), while linear acceleration and angular velocity are read from the raw registers, whose scale factors are more reliable than the DMP FIFO's. A one-shot 200-sample bias calibration runs at boot with the car stationary.
- **EKF (`robot_localization`).** Fuses the front-wheel Hall odometry (`vx` only) with the IMU's absolute DMP yaw and yaw rate at 20 Hz in 2D mode, publishing the `odom → base_link` transform. This fused yaw is what the reactive controller's heading-hold anchors to when the track lines are lost — it doesn't drift the way a raw gyro integration would over a multi-second blind spot like a ramp.

## Control loops

Three nested loops turn a lane-following goal into motor and servo commands:

1. **Outer — trajectory (`speedy_navigation`).** A PID on lateral and heading error (plus curvature feedforward) proposes a steering angle from the line detector; the mini-DWA layer vetoes any of ~20 candidate arcs that a LiDAR scan says would collide, and the closest surviving arc to the vision proposal wins.
2. **Middle — kinematics (`ros2_control`).** In `AUTO`, the `bicycle_steering_controller` converts that steering angle + speed into individual joint commands via Ackermann kinematics. In `MANUAL`, `manual_steering_controller`/`manual_drive_controller` pass the joystick through directly. `speedy_supervisor` switches between the two controller sets **atomically** (`SwitchController` with `STRICT` strictness) on a joystick chord, and only updates its own state once the switch is confirmed — never optimistically.
3. **Inner — actuation (`speedy_control`).** The hardware interface closes a velocity PID (`kp=1.0`, `ki=0.4`) with feedforward against Hall-encoder-derived speed (exponential filter, zero-velocity timeout for a clean stop), and maps the commanded steering angle to a servo pulse through a **quadratic regression** (`pulse = a0 + a1·deg + a2·deg²`) rather than a linear one — fit by driving the car and relating commanded pulse to the physical angle estimated from measured yaw rate and speed (bicycle model), which corrected roughly 3° of steering backlash asymmetry that a linear mapping couldn't.

## Dataset and model training

- [`speedy_dataset`](iot/software/src/speedy_dataset) captures single or burst JPEG frames on a joystick button press, **only while in `MANUAL`** (auto-unsubscribes otherwise to save CPU), with an optional live undistort/grayscale pass and a background writer thread so bursts don't stall the ROS executor. These frames seeded the Roboflow dataset (525 images, 5 classes: `BOX`, `FLOOR`, `WALL`, `RAMP`, `TOP`) used to train the obstacle model — see [`yolo/README.roboflow.txt`](yolo/README.roboflow.txt).
- [`yolo/train_yolo.py`](yolo/train_yolo.py) fine-tunes a **YOLO11n** at 320px (a quarter of the compute of 640px, still enough for large objects like the ramp and boxes) and exports it to **NCNN** for ARM inference.
- On-device, [`yolo_ncnn.py`](iot/software/src/speedy_vision/speedy_vision/yolo_ncnn.py) runs inference with plain NumPy/OpenCV — no PyTorch on the robot — doing the letterbox, decode, and NMS itself. Inference is pinned to a single thread so it can never starve the Pi 4's control loop.

## Telemetry

The Foxglove bridge is tuned against buffer bloat over the robot's WiFi hotspot: a 2 MB send buffer and a QoS depth of 1 mean the bridge always drops backlog in favor of the newest frame rather than queuing, and the video feed is the camera's already-JPEG-compressed stream — never the raw `image_raw` topic — so the WebSocket never burns Pi 4 CPU re-encoding it.

## Requirements

| Tool      | Minimum version              |
| --------- | ---------------------------- |
| ROS 2     | Jazzy                        |
| Ansible   | 2.15+                        |
| Distrobox | latest (PC development only) |
| Python    | 3.12+ (YOLO training)        |

## How to run

```bash
git clone --recursive https://github.com/nycocado/speedy.git
```

**Robot provisioning** (Raspberry Pi, from a fresh Raspberry Pi OS Lite install):

```bash
cd iot/ansible
ansible-playbook -i inventory.ini raspberrypi/main.yml
```

This installs ROS 2 Jazzy natively, syncs and builds the workspace, and registers the `speedy.service` systemd unit that auto-launches `speedy_bringup` on boot.

**PC development environment** (Distrobox container mirroring the robot's workspace):

```bash
ansible-playbook -i inventory.ini distrobox/main.yml
```

**Manual launch** (already-provisioned robot):

```bash
source /opt/ros/jazzy/setup.bash
source ~/speedy_ws/install/setup.bash
ros2 launch speedy_bringup speedy.launch.py
```

**YOLO model training** ([`yolo/`](yolo)):

```bash
cd yolo
python train_yolo.py
```

## Repository structure

```
speedy/
├── iot/
│   ├── software/src/    # ROS 2 Jazzy workspace — speedy_* packages + vendored deps/
│   └── ansible/         # Robot provisioning, host setup, and Distrobox dev environment
├── yolo/                # Roboflow dataset config and YOLO11n training script
├── media/               # Milestone reports, slides, BOM, and circuit diagrams
└── LICENSE
```

## Documentation

#### Milestone 1

- [Report](media/milestone-1/report.pdf) — initial hybrid Pi + ESP32-S3 architecture, requirements, and BOM.
- [Slides](media/milestone-1/slides.pdf)
- [Circuit diagram](media/milestone-1/circuit.pdf)
- [Component diagram](media/milestone-1/component.pdf)
- [BOM](media/milestone-1/bom.xlsx)

#### Milestone 2

- [Circuit diagram](media/milestone-2/circuit.pdf) — final standalone Raspberry Pi 4 architecture.
- [Component diagram](media/milestone-2/component.pdf)
- [BOM](media/milestone-2/bom.xlsx)
- [Demo video](media/milestone-2/video.mp4)

#### Photos

- [Studio and analog photos](media/photos) — close-up shots of the chassis and electronics.

## Team

- [**Nycolas Souza**](https://github.com/nycocado) — firmware (C++ hardware interface), computer vision pipeline, navigation controllers.
- [**Kira Sousa**](https://github.com/Kira-Sousa) — dataset training and Roboflow session management, YOLO11n → NCNN export.
- [**Luan Ribeiro**](https://github.com/Ninjaok) — hardware engineering, steering calibration (bicycle-model regression), chassis assembly.
- [**Lohanne Guedes**](https://github.com/lohanneguedes) — physical prototyping, electronics wiring, network and hotspot infrastructure.

## License

Distributed under the **CC BY-NC 4.0** license, © 2026 Nycolas Souza, Luan Ribeiro, Lohanne Guedes, Kira Sousa.

Non-commercial use only. You may share and adapt the material as long as you give appropriate credit and do not use it for commercial purposes.

The full text is in [LICENSE](LICENSE).
