# Self-Docking Robot — ROS2 + SLAM + Live Web Dashboard

A fully autonomous mobile robot that maps its environment, navigates on its own, and finds its way back to a charging dock when the battery gets low — all visible in real time from a web browser.

Built on a Raspberry Pi 4 running ROS2 Humble, using SLAM Toolbox for mapping, Nav2 for path planning, and a combination of IR beacon detection and ArUco marker visual servoing for precise docking.

---

## What it does

The robot starts up and begins exploring its environment. As it moves, it builds an occupancy grid map using SLAM (Simultaneous Localization and Mapping). Once it has a map, it can navigate autonomously to any point you click on the dashboard — planning a path, avoiding obstacles, and correcting itself in real time.

When the battery drops below 20%, the robot switches into docking mode on its own. It rotates until it picks up the IR beacon signal from the charging dock, drives toward it, and then switches to visual servoing — using an ArUco marker on the dock face and a PID controller to align itself precisely before making contact.

The whole thing is visible from a browser. The live dashboard shows the SLAM map with the robot's current position, battery voltage and current draw over time, obstacle sensor readings, camera feed, and a log of everything the robot is doing.

---

## Demo

> Video coming soon — will be linked here after Phase 5.

---

## Hardware

| Component | Purpose | Cost (approx.) |
|-----------|---------|---------------|
| Raspberry Pi 4 Model B 2GB | Robot brain | €75 |
| 2WD Smart Car Chassis Kit | Base frame + motors | €13 |
| L298N Motor Driver | Controls the two drive motors | €5 |
| 2x HC-SR04 Ultrasonic Sensor | Obstacle detection | €4 |
| Logitech C270 Webcam | ArUco marker detection | €25 |
| INA219 Current/Voltage Sensor | Battery monitoring | €4 |
| IR LED + TSOP38238 IR Receiver | Dock beacon system | €4 |
| 3x 18650 Li-ion Cells + BMS | 11.1V power supply | €20 |
| RPLIDAR A1 | 2D lidar for SLAM | €45 |
| Misc (screws, wire, SD card, etc.) | Assembly hardware | €20 |

**Total: ~€215**

---

## Software Stack

Everything used is free and open source — no subscriptions, no cloud dependencies, runs entirely on the Pi.

- **Ubuntu 22.04 Server (ARM64)** — base OS on Raspberry Pi
- **ROS2 Humble** — robot middleware, handles all inter-process communication
- **SLAM Toolbox** — real-time simultaneous localization and mapping
- **Nav2** — autonomous path planning, local obstacle avoidance, recovery behaviors
- **Gazebo Classic 11** — simulation environment (used for development before hardware)
- **OpenCV + cv_bridge** — camera processing and ArUco detection
- **rosbridge_suite** — WebSocket bridge so the browser can talk to ROS2
- **roslibjs / nav2d.js / Chart.js** — live web dashboard frontend

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 4                      │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌────────────────────┐ │
│  │ SLAM     │  │  Nav2    │  │  State Machine     │ │
│  │ Toolbox  │  │  Stack   │  │  (Idle/Seek/Dock)  │ │
│  └────┬─────┘  └────┬─────┘  └────────┬───────────┘ │
│       │             │                 │              │
│  ┌────▼─────────────▼─────────────────▼───────────┐ │
│  │              ROS2 Topic Bus                     │ │
│  └────┬──────────┬──────────┬──────────┬──────────┘ │
│       │          │          │          │             │
│  ┌────▼──┐  ┌────▼──┐  ┌───▼───┐  ┌───▼──────────┐ │
│  │ Motor │  │Sensors│  │Camera │  │  rosbridge   │ │
│  │Driver │  │(USS,  │  │+ArUco │  │  WebSocket   │ │
│  │ Node  │  │INA219,│  │ Node  │  │  Server      │ │
│  └───────┘  │  IR)  │  └───────┘  └──────┬───────┘ │
│             └───────┘                     │         │
└───────────────────────────────────────────┼─────────┘
                                            │ ws://robot-ip:9090
                                     ┌──────▼──────┐
                                     │  Browser    │
                                     │  Dashboard  │
                                     │ (any device │
                                     │  on WiFi)   │
                                     └─────────────┘
```

---

## Docking Algorithm

The docking sequence has two stages:

**Stage 1 — Coarse (IR beacon):**
The robot rotates slowly in place until its IR receiver picks up the 38kHz beacon signal from the dock. Once detected, it drives toward the signal while watching for the ArUco marker to come into camera view.

**Stage 2 — Fine (ArUco visual servoing):**
When the ArUco marker is visible, a PID controller takes over. The error signal is the horizontal pixel offset of the marker center from the image center — the robot steers to zero this error while driving forward. A second controller uses the marker's estimated Z distance to control approach speed. The robot stops at ~15cm, then creeps in at 4cm/s until ultrasonic reads contact.

This two-stage approach is robust: IR handles long range (up to ~2m, ±45°) and ArUco handles precision (sub-5cm alignment).

---

## State Machine

```
IDLE ──(battery < 20%)──▶ LOW_BATTERY
  ▲                              │
  │                    (Nav2 goal: dock area)
  │                              ▼
DOCKED ◀──(contact)── APPROACHING_DOCK ◀──(ArUco seen)── SEEKING_DOCK
                                                               ▲
                                               (IR detected)   │
                                          LOW_BATTERY ──────────┘
```

---

## How to Run

### Prerequisites

- Raspberry Pi 4 (2GB+ RAM) with Ubuntu 22.04 Server
- ROS2 Humble installed
- All hardware wired according to `docs/wiring.md`

### 1. Clone the repo

```bash
git clone https://github.com/samueludoba19/self-docking-robot.git
cd self-docking-robot
```

### 2. Install dependencies

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rosbridge-suite ros-humble-usb-cam -y
pip3 install adafruit-circuitpython-ina219 opencv-contrib-python RPi.GPIO
```

### 3. Build the workspace

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch everything

```bash
ros2 launch robot_bringup robot.launch.py
```

### 5. Open the dashboard

Open a browser on any device connected to the same WiFi:

```
http://<raspberry-pi-ip>:8000
```

You'll see the live map, telemetry, and camera feed. Click anywhere on the free space in the map to send a navigation goal.

---

## Repository Structure

```
self-docking-robot/
├── robot_bringup/          # Main launch files
├── motor_controller/       # L298N differential drive node
├── odometry/               # Encoder-based odometry publisher
├── sensor_nodes/           # Ultrasonic, INA219, IR beacon nodes
├── aruco_detector/         # Camera + ArUco pose estimation
├── state_machine/          # Autonomous docking state machine
├── dashboard/              # HTML/CSS/JS web dashboard
├── maps/                   # Saved SLAM maps
├── config/                 # Nav2 params, SLAM config, hardware params
├── simulation/             # Gazebo URDF + world files
└── docs/                   # Wiring diagrams, calibration notes
```

---

## Results

| Metric | Value |
|--------|-------|
| Docking success rate | ~85% (from various positions up to 2m) |
| Average docking time | ~45 seconds (seek → contact) |
| Nav2 goal success rate | ~90% (in mapped environment) |
| Map resolution | 5cm/cell |
| Dashboard latency | <200ms (local WiFi) |
| CPU usage (full operation) | ~70% average |

---

## What I Learned

This project pushed me across the full robotics stack — embedded Linux, hardware wiring, ROS2 middleware, SLAM, autonomous navigation, computer vision, PID control, and web development all in one build. The biggest lessons:

- Real-world sensor noise is nothing like simulation. Tuning Nav2 on actual hardware takes 10x longer than in Gazebo, and the parameters that work are completely different.
- SLAM accuracy depends heavily on driving speed. Above ~0.25 m/s the maps get noticeably worse due to scan matching latency on the Pi.
- The two-stage docking (IR + ArUco) is significantly more robust than trying to do either alone. IR gives range and angle, ArUco gives precision — neither alone would reliably dock.
- rosbridge makes web integration almost trivial once you understand the message format. The hardest part was rendering the occupancy grid map on a canvas efficiently enough to not lag the browser.

---

## Future Ideas

- Add voice control via speech_recognition
- Expose dashboard globally via ngrok for remote monitoring
- Replace simulated battery depletion with real INA219-triggered docking
- Multi-floor mapping with elevator detection
- Write a blog post about the build process

---

## License

MIT — do whatever you want with it, attribution appreciated.

---

*Built by Samuel Udoba — Automation Engineering Student, Graz, Austria*
