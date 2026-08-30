# NIDAR RescueSwarm — Autonomous Multi-Drone Search & Rescue System

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Aerostack2](https://img.shields.io/badge/Platform-Aerostack2-purple.svg)](https://github.com/aerostack2/aerostack2)
[![React](https://img.shields.io/badge/GCS-React%2019-cyan.svg)](https://react.dev/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**NIDAR RescueSwarm** is an autonomous multi-drone aerial system engineered for high-stress search and rescue (SAR) operations in GPS-denied or degraded environments. The platform combines autonomous swarm sweep coverage, real-time computer vision survivor detection, and an advanced Ground Control Station (GCS) with complete testing parity across **Simulation** (Gazebo / SITL) and **Hardware** (HITL / Physical Drones).

---

## 🌟 Key Features & Architecture

```
                                  +---------------------------------------+
                                  |    Ground Control Station (GCS)       |
                                  | React 19 + TypeScript + Obsidian Dark |
                                  +-------------------+-------------------+
                                                      | ROS WebSocket Bridge
                                                      v
                                  +-------------------+-------------------+
                                  |     ROS 2 Swarm Orchestrator      |
                                  +---------+-----------------+-----------+
                                            |                 |
                   +------------------------+                 +------------------------+
                   |                                                                   |
                   v                                                                   v
+------------------+------------------+                             +------------------+------------------+
|      nidar_mission_executor         |                             |         nidar_detection         |
| Zone Allocation & Sweep Coverage   |                             | YOLOv8 Thermal / RGB + ByteTrack |
+------------------+------------------+                             +------------------+------------------+
                   |                                                                   |
                   v                                                                   v
+------------------+-------------------------------------------------------------------+------------------+
|                                    Aerostack2 Aerial Autonomy Platform                                  |
|                              (Quadrotor Control, Telemetry & Swarm Interfaces)                          |
+---------------------------------------------------------------------------------------------------------+
```

### 1. Ground Control Station (GCS) SPA
- **Cockpit Aesthetic**: Built with the **Obsidian Command** dark visual design (`#0a0d12` surface, translucent glassmorphism cards, `#7e14ff` glowing accents, and monospaced JetBrains Mono data typography).
- **Simulation & Hardware Testing Parity**: Operator interface features identical telemetry panels, tactical map overlays, video feeds, manual flight controls, and log consoles whether testing standalone in browser, in Gazebo simulation, or on real flight hardware.
- **5 Core View Modes**:
  1. **Mission Control**: Tactical map, swarm status cards, live detection counter, quick launch/recall/abort commands.
  2. **Mission Planning**: Manual boundary drawing, **"GENERATE RANDOM ARENA"** automatic boundary generator, **12 ft x 12 ft Launch/Landing Station** manager, random launch placement, and survivor dummy scatter.
  3. **Manual Flight Ops**: Individual drone target selector (`ALL`, `drone0`..`drone3`), altitude & speed setpoint sliders, D-Pad manual joystick nudge, and critical emergency cut-motors override.
  4. **Video & Detections**: 4-camera video feed grid with synthetic heat signature canvas streams (Thermal IR & EO mode toggle), target bounding boxes, and enlarged viewer.
  5. **Console Logs**: Terminal diagnostic console with log severity filters (`INFO`, `WARN`, `ERROR`, `CMD`), source selectors, and auto-scroll.
- **Map Tile Switcher**: Toggle between **High-Res Esri World Satellite Imagery** and **CartoDB Dark Tactical Vector** tiles.

### 2. Fixed 12 ft x 12 ft Launch & Landing Zone
- **Competition Rule Compliance**: Every quadrotor in the swarm launches from and lands within a fixed **12 ft x 12 ft (3.65m x 3.65m)** launch/landing box centered on the Home launch site.
- The GCS map renders a glowing green box with corner reticles and dimensional annotations, enforcing launch position boundaries before mission start.

### 3. Autonomous Mission Planning & Detection
- **Zone Splitting & Sweep Coverage**: Automatically divides search area polygons into optimal coverage zones per drone (`drone0` through `drone3`).
- **YOLOv8 + ByteTrack Perception**: Real-time person and survivor identification using thermal infrared and RGB camera feeds.

---

## 🛠️ Repository Structure

```
NIDAR/
├── gcs/                               # React + TypeScript Ground Control Station
│   ├── src/
│   │   ├── components/                # UI Components (HeaderNavbar, MapView, VideoPanel, etc.)
│   │   ├── mission/                   # Launch Site & 12ft Box Manager
│   │   ├── ros/                       # ROSLIB WebSockets Hooks & Telemetry Subscribers
│   │   └── types/                     # TypeScript Interfaces (GCS, Drone, Mission)
├── framework_ws/                      # ROS 2 Workspace
│   └── src/
│       ├── nidar_mission_executor/    # Swarm Zone Splitting & Lawnmover Coverage Node
│       ├── nidar_detection/           # YOLOv8 + ByteTrack Object Tracking Node
│       └── nidar_gcs_bridge/          # ROSLIB WebSocket Bridge Node
├── project_gazebo/                    # Gazebo Simulation Models & World Configs
└── DOCUMENTS/
    └── NIDAR_Implementation_Plan.md   # Primary Architecture Implementation Plan
```

---

## 🚀 Quick Start Guide

### 1. Launching the Ground Control Station (GCS)
```bash
cd gcs
npm install
npm run dev -- --port 5173
```
Open **`http://localhost:5173`** in your browser to access the GCS interface.

### 2. Building the ROS 2 Workspace
```bash
cd framework_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🙏 Credits & Acknowledgments

This project incorporates and builds upon the following open-source frameworks:

* **[Aerostack2](https://github.com/aerostack2/aerostack2)** — We gratefully acknowledge the **Aerostack2** team and contributors for their outstanding aerial robotics framework. Aerostack2 provides the foundational multi-drone flight control primitives, telemetry abstractions, and platform interfaces that power our swarm autonomy pipeline.
* **[ROS 2 (Humble Hawksbill)](https://docs.ros.org/en/humble/)** — Robot Operating System middleware for distributed node communication.
* **[Leaflet](https://leafletjs.com/)** — Interactive mapping library powering our tactical GCS map view.

---

## 📄 Documentation

For full technical specifications, mission file schemas, and phase-by-phase development protocols, refer to:
* [`DOCUMENTS/NIDAR_Implementation_Plan.md`](file:///home/ayush/NIDAR/DOCUMENTS/NIDAR_Implementation_Plan.md)
