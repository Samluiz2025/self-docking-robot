# Self-Docking Robot (ROS2 Humble · Raspberry Pi 4)

An autonomous differential-drive robot that detects its charging dock with computer vision and is being developed to dock itself when its battery runs low. Built on a Raspberry Pi 4 with ROS2 Humble, structured as modular packages for motor control, sensing, vision and state management.

> **Status: in active development.** Core hardware nodes, ArUco pose estimation and the docking state machine are implemented. Nav2 integration and physical docking trials are in progress. See [Project Status](#project-status) for details.

---

## Project Status

| Component | Status |
|---|---|
| Modular ROS2 package structure | ✅ Done |
| Motor control (L298N differential drive) | ✅ Done |
| Ultrasonic obstacle detection (2× HC-SR04, 30 cm range) | ✅ Working, tuning in progress |
| ArUco marker detection + 3D pose estimation (OpenCV, `cv2.solvePnP`) | ✅ Done |
| Battery monitoring over I2C (INA219) | ✅ Done |
| 5-state docking state machine | ✅ Implemented |
| Wheel odometry | 🔧 In progress |
| Gazebo simulation (URDF) | 🔧 In progress |
| Web dashboard | 🔧 In progress |
| Nav2 navigation | 🔧 In progress |
| Physical docking trials | 📋 Next step |
| SLAM mapping with 2D lidar | 📋 Planned |
| IR beacon for long-range dock finding | 📋 Planned |
| Demo video and measured results | 📋 After docking trials |

---

## How It Works

1. **Normal operation:** the robot drives and avoids obstacles using two front ultrasonic sensors.
2. **Battery monitoring:** an INA219 sensor reads battery voltage over I2C. When it drops below a threshold, the state machine switches to docking mode.
3. **Finding the dock:** the camera searches for an ArUco marker mounted on the dock.
4. **Pose estimation:** once the marker is visible, OpenCV and `cv2.solvePnP` estimate the dock's 3D position and orientation relative to the robot.
5. **Docking:** the robot uses that pose to align and approach the dock, then charges and undocks when finished.

### State Machine

```
IDLE ──(low battery)──▶ SEEKING_DOCK ──(marker found)──▶ DOCKING
  ▲                                                        │
  │                                                    (contact)
  │                                                        ▼
  └────────────(charged)──── UNDOCKING ◀──(charged)──── CHARGING
```

---

## Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi 4 Model B | Main computer |
| 2WD chassis with DC motors | Robot base |
| L298N motor driver | Drives both motors |
| 2× HC-SR04 ultrasonic sensors | Obstacle detection |
| Logitech C270 webcam | ArUco marker detection |
| INA219 current/voltage sensor | Battery monitoring (I2C) |
| 3× 18650 Li-ion cells + BMS | Power supply |

Planned additions: RPLIDAR A1 (for SLAM) and an IR LED + TSOP38238 receiver (for long-range dock detection).

---

## Software Stack

- **Ubuntu 22.04 Server (ARM64)** on the Raspberry Pi
- **ROS2 Humble** for communication between nodes
- **OpenCV** for camera processing and ArUco pose estimation
- **Nav2** for navigation (integration in progress)
- **Gazebo Classic** for simulation (in progress)

---

## Repository Structure

```
self-docking-robot/
├── robot_bringup/      # Launch files
├── motor_controller/   # L298N differential drive node
├── odometry/           # Wheel odometry (in progress)
├── sensor_nodes/       # Ultrasonic and INA219 nodes
├── aruco_detector/     # Camera + ArUco pose estimation
├── state_machine/      # Docking state machine
├── dashboard/          # Web dashboard (in progress)
├── simulation/urdf/    # Robot model for Gazebo
└── docs/               # Wiring and notes
```

---

## How to Run

**Requirements:** Raspberry Pi 4 with Ubuntu 22.04 Server, ROS2 Humble, hardware wired as described in `docs/`.

```bash
# Clone into a ROS2 workspace
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/Samluiz2025/self-docking-robot.git

# Python dependencies
pip3 install adafruit-circuitpython-ina219 opencv-contrib-python RPi.GPIO

# Build
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch robot_bringup robot.launch.py
```

---

## Roadmap

1. Finish Nav2 integration and tune obstacle avoidance on real hardware
2. Run physical docking trials and record success rate and docking time
3. Add a demo video
4. Add 2D lidar and SLAM mapping
5. Add IR beacon for finding the dock from longer distances

Results will be added here once they are measured on the real robot.

---

## What I'm Learning

- Structuring a robot as separate ROS2 packages that communicate through topics
- Turning camera images into usable 3D position data with ArUco markers and `solvePnP`
- Designing a state machine that reacts to real sensor readings
- How much harder tuning on real hardware is compared to the plan

---

*Built by Samuel Udoba, Smart Automation student at FH Campus 02, Graz, Austria. License: MIT.*
