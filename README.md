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

# Design & URDF

<table>
<tr>
<td align="center" width="50%">

### Fusion 360 Design

<img src="design/robot.png" width="430">

</td>

<td align="center" width="50%">

### URDF in RViz

<img src="media/robot_in_rviz.png" width="430">

</td>
</tr>
</table>

The chassis was modeled in **Autodesk Fusion 360**. The design is inspired by **Arjuna**, a robot from the robotics startup **NEWRRO**, adapted into a four-wheel mecanum platform.

The Fusion 360 model was converted into a **URDF/Xacro**, defining every link, joint, wheel, LiDAR mount, and IMU mount so the robot simulates accurately in Gazebo and visualizes correctly in RViz2.

---

# Simulation Results

<table>
<tr>
<td align="center" width="50%">

### Gazebo Simulation

<img src="media/robot_in_gazebo.png" width="430">

</td>

<td align="center" width="50%">

### Generated Map

<img src="media/map_in_rviz.png" width="430">

</td>
</tr>
</table>

The robot runs inside **Gazebo Harmonic**, while **SLAM Toolbox** generates a real-time occupancy grid map for autonomous navigation.

---

## Tech Stack

| Layer | Technology |
|-------|------------|
| Robot framework | ROS2 Jazzy |
| Simulation | Gazebo Harmonic |
| Robot description | URDF / Xacro |
| Drive | Four-wheel Mecanum |
| Sensors | Simulated LiDAR |
| Odometry | Custom Python Forward Kinematics |
| Mapping | SLAM Toolbox |
| Navigation | Nav2 |
| Visualization | RViz2 |
| Bridge | ros_gz_bridge |

---

## Project Structure

```text
mecanum-nav-robot/
├── design/
├── docs/
├── media/
├── src/
│   ├── robo_desc/
│   ├── mecanum_odom/
│   ├── robo_bringup/
│   └── auto_patrol/
└── README.md
```

---

# How it Works

### Custom Mecanum Odometry

Gazebo's default odometry plugin is inaccurate for this robot because the mecanum wheels use simplified collision spheres.

Instead, a custom Python node computes forward kinematics directly from wheel joint states:

```text
vx = r/4 (wlf + wrf + wlb + wrb)

vy = r/4 (-wlf + wrf + wlb - wrb)

wz = r/(4(L+W)) (-wlf + wrf - wlb + wrb)
```

The computed odometry is published to ROS2 and consumed by **SLAM Toolbox** and **Nav2**.

---

## Launch Sequence

```text
0 s  → Gazebo starts
0 s  → Robot State Publisher
5 s  → Robot spawned
6 s  → ros_gz_bridge
7 s  → Custom Odometry Node
9 s  → RViz2
```

---

# Autonomous Navigation

<table>
<tr>
<td align="center" width="50%">

### Navigation

<img src="media/nav.png" width="430">

</td>

<td align="center" width="50%">

### Goal Reached

<img src="media/goal.png" width="430">

</td>
</tr>
</table>

The robot autonomously plans and follows collision-free paths using the **Nav2 navigation stack**, successfully reaching the selected goal pose.

---

## Getting Started

### Install Dependencies

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-* ros-jazzy-ros-gz-bridge
```

### Clone

```bash
git clone https://github.com/nandanaa555/mecanum-nav-robot.git

cd mecanum-nav-robot

colcon build

source install/setup.bash
```

### Mapping

```bash
ros2 launch robo_bringup slam.launch.py
```

Open another terminal:

```bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Navigation

```bash
ros2 launch robo_bringup navigation.launch.py
```

### Autonomous Patrol

```bash
ros2 launch robo_bringup auto_patrol_nav.launch.py
```

---

# Demo Videos

- 🎥 [Simulation Demo](media/demo.webm)
- 🎥 [SLAM Mapping](media/mapping.webm)
- 🎥 [Waypoint Navigation](media/way_point_nav.webm)
- 🎥 [Goal Navigation](media/goal_navigation.webm)

---

# Author

**Nandanaa M S**

📧 nandanaams555@gmail.com

🌐 https://github.com/nandanaa555

📍 Bengaluru, Karnataka

---

# License

MIT License
