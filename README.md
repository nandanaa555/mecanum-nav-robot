<div align="center">

# Mecanum Nav Robot

**An omnidirectional mobile robot that maps and navigates its environment autonomously — built on ROS2, simulated in Gazebo.**

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?logo=gazebo&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-ready-1D9E75)
![License](https://img.shields.io/badge/license-MIT-blue)

</div>

---

Differential-drive robots can only move forward, backward, and turn in place — any sideways motion means stopping and rotating first. This project uses a **four-wheel mecanum drive base** instead: angled rollers on each wheel let the robot move forward, strafe sideways, and rotate independently, all without changing its heading.

Built end to end in this repo: a custom robot design, its URDF, a Gazebo simulation, a hand-written odometry node solving a real mecanum-in-simulation problem, SLAM-based mapping, and a full Nav2 navigation and autonomous patrol stack.

---

## Design

<div align="center">
<img src="design/robot.png" width="720" alt="Robot design in Fusion 360">
</div>

The chassis was modeled in **Autodesk Fusion 360**. The design is inspired by *Arjuna*, a robot from the robotics startup **NEWRRO**, adapted here into a four-wheel mecanum base for this project.

## URDF

<div align="center">
<img src="media/robot_in_rviz.png" width="720" alt="Robot URDF rendered in RViz">
</div>

The Fusion 360 model was converted into a **URDF** (via Xacro), defining every link, joint, wheel, and sensor mount — LiDAR and IMU included — so the robot simulates accurately in Gazebo and visualizes correctly in RViz2.

## Gazebo Simulation

<p align="center">
  <img src="media/robot_in_gazebo.png" width="800">
</p>
---

## Tech stack

| Layer              | Technology                                       |
| ------------------ | ------------------------------------------------- |
| Robot framework     | ROS2 Jazzy                                        |
| Simulation          | Gazebo Harmonic (`gz sim`)                        |
| Robot description   | URDF / Xacro                                      |
| Drive type          | Mecanum wheel — 4WD omnidirectional               |
| Sensing             | Simulated LiDAR                                   |
| Odometry            | Custom mecanum forward-kinematics node (Python)   |
| Mapping             | SLAM Toolbox                                      |
| Navigation          | Nav2                                              |
| Visualization       | RViz2                                             |
| 3D design           | Autodesk Fusion 360                               |
| ROS↔Sim bridge      | `ros_gz_bridge`                                   |

---

## Project structure

Four ROS2 packages, each with a single responsibility:

```
mecanum-nav-robot/
├── design/                          # Fusion 360 design exports/renders
├── docs/                            # URDF screenshots and reference docs
├── media/                           # Demo videos
├── src/
│   ├── robo_desc/                   # Robot description + Gazebo simulation
│   │   ├── urdf/robot.xacro         # Robot description (URDF/Xacro)
│   │   ├── launch/gazebo.launch.py  # Main simulation launch
│   │   ├── launch/display.launch.py # RViz-only launch (URDF, no sim)
│   │   ├── world/nav_world.sdf      # Simulation world (SDF)
│   │   ├── config/ros_gz_bridge_gazebo_2.yaml
│   │   └── meshes/                  # Custom STL meshes (wheels, body, LiDAR)
│   │
│   ├── mecanum_odom/                # Custom odometry package
│   │   └── mecanum_odom/odom_node.py
│   │
│   ├── robo_bringup/                # SLAM + Nav2 bringup package
│   │   ├── config/mapper_params_online_async.yaml   # SLAM Toolbox params
│   │   ├── config/nav2_params.yaml                  # Nav2 stack params
│   │   ├── launch/slam.launch.py                    # Sim + online SLAM mapping
│   │   ├── launch/navigation.launch.py               # Sim + Nav2 navigation
│   │   ├── launch/auto_patrol_nav.launch.py          # Navigation + autonomous patrol
│   │   └── maps/my_map.pgm, my_map.yaml              # Saved occupancy grid map
│   │
│   └── auto_patrol/                 # Autonomous patrol behavior package
│       └── auto_patrol/patrol.py    # Sends the robot on a patrol route via Nav2
│
└── README.md
```

---

## How it works

### Mecanum wheel odometry

The robot uses **four mecanum wheels** — angled-roller wheels that let it move in any direction (forward, sideways, diagonal) without rotating the body.

**Why a custom odometry node?** Gazebo's built-in odometry plugin relies on wheel collision geometry to compute motion. This robot's mecanum wheels use a **sphere as the collision primitive** — a common simplification for the angled-roller contact point — which makes Gazebo's default odometry inaccurate for this drive type.

To fix that, `odom_node.py` bypasses Gazebo's plugin entirely and computes odometry straight from **joint states** using mecanum forward kinematics:

```
vx  = r/4 * ( w_lf + w_rf + w_lb + w_rb )            ← forward / backward
vy  = r/4 * (-w_lf + w_rf + w_lb - w_rb )            ← left / right strafe
wz  = r / (4*(L+W)) * (-w_lf + w_rf - w_lb + w_rb)   ← rotation
```
`r` = wheel radius · `L` = half wheelbase · `W` = half wheel separation

This gives accurate real-time position and heading, which feeds directly into SLAM.

### Launch sequence

```
0s → Gazebo starts with the simulation world
0s → Robot State Publisher starts
5s → Robot spawns in Gazebo
6s → ROS–Gazebo bridge starts (connects topics)
7s → Mecanum odometry node starts
9s → RViz2 opens for visualization
```

---

## Getting started

### Prerequisites

- Ubuntu 22.04 / 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- `slam_toolbox`

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-* ros-jazzy-ros-gz-bridge
```

### Clone and build

```bash
git clone https://github.com/nandanaa555/mecanum-nav-robot.git
cd mecanum-nav-robot
colcon build
source install/setup.bash
```

### Map an environment

```bash
ros2 launch robo_bringup slam.launch.py
```
In a new terminal:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Save the map once mapping is complete:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Navigate autonomously

```bash
ros2 launch robo_bringup navigation.launch.py
```

### Run the autonomous patrol behavior

```bash
ros2 launch robo_bringup auto_patrol_nav.launch.py
```

---

## Generated map


<p align="center">
  <img src="media/map_in_rviz.png" width="800">
</p>

The robot successfully maps the warehouse environment using SLAM Toolbox.
The robot successfully maps its environment as a 2D occupancy grid (`my_map.pgm` + `my_map.yaml`):

- **Resolution:** 0.05 m/pixel (5 cm per pixel)
- **White** = free space · **Black** = walls/obstacles · **Grey** = unknown
---

## Navigation

<p align="center">
  <img src="media/nav.png" width="800" alt="Autonomous Navigation">
</p>

The robot autonomously navigates to the selected destination using the Nav2 stack.

---

## Goal Navigation

<p align="center">
  <img src="media/goal.png" width="800" alt="Goal Navigation">
</p>

The robot successfully reaches the goal pose while avoiding obstacles using global and local path planning.
---

## Demo videos

📹 **[Main simulation walkthrough](media/demo.webm)** — robot spawning, SLAM mapping in progress, and the final map in RViz.

More recordings from testing and development:
- [Video 2](media/video2.webm)
- [Video 3](media/video3.webm)

*(Browse the full set in [`media/`](media/).)*

---

## Author

**Nandanaa M S**
[GitHub](https://github.com/nandanaa555) · nandanaams555@gmail.com · Bengaluru, Karnataka

## License

MIT License — feel free to use and build on this project.
