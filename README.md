# 🏭 Warehouse Robot Simulation — Autonomous Mapping with Mecanum Drive

> **Rookies Hackathon 2026** | Autonomous robot spawned in a Gazebo warehouse environment, performing SLAM-based mapping using a custom mecanum-wheeled robot built in ROS2.

---

## 🎯 Problem Statement

Modern warehouses are large, complex environments where human workers spend significant time navigating and mentally mapping the space. Before any autonomous robot can perform tasks like delivery, inventory tracking, or inspection, it first needs to **know the map of its environment**.

Manually creating warehouse floor maps is time-consuming and error-prone. The challenge: **can a robot autonomously explore a warehouse and generate an accurate 2D map — entirely on its own?**

---

## 💡 Our Solution

We designed and simulated a **4-wheeled mecanum drive robot** in ROS2 + Gazebo Harmonic that:

- Spawns inside a realistic 3D industrial warehouse environment
- Uses a **LiDAR sensor** to perceive the surroundings
- Computes **wheel odometry** from joint states using mecanum forward kinematics
- Publishes odometry and TF transforms in real time
- Enables **SLAM (Simultaneous Localization and Mapping)** to generate a complete 2D occupancy grid map of the warehouse

The final output is a `.pgm` + `.yaml` map file that can be directly fed into a **Nav2 navigation stack** for autonomous robot operation in future stages.

---

## 🌍 Real-World Application

This project is the **foundation layer** for a fully autonomous warehouse robot system. Once the map is generated, the same robot can be used for:

| Application | How This Project Enables It |
|---|---|
| 📦 Autonomous Inventory Delivery | Nav2 uses the generated map for path planning |
| 🔍 Warehouse Inspection Bot | Robot patrols known map area autonomously |
| 🚨 Safety Monitoring | LiDAR detects unexpected obstacles in real-time |
| 📍 Asset Tracking | Robot localizes itself within the known map |
| 🤝 Human-Robot Collaboration | Safe navigation around workers using costmaps |

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Robot Framework | ROS2 Jazzy |
| Simulation | Gazebo Harmonic (gz sim) |
| Robot Description | URDF / Xacro |
| Drive Type | Mecanum Wheel (4WD omnidirectional) |
| Sensing | LiDAR (simulated) |
| Odometry | Custom mecanum forward kinematics node (Python) |
| Mapping | SLAM Toolbox |
| Visualization | RViz2 |
| 3D Design | Autodesk Fusion 360 |
| Bridge | ros_gz_bridge (ROS2 ↔ Gazebo topic bridge) |

---

## 📁 Project Structure

```
v_hack_ws/
├── src/
│   ├── robo_desc/                  # Main robot package
│   │   ├── urdf/
│   │   │   └── robot.xacro         # Robot description (URDF/Xacro)
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py    # Main simulation launch file
│   │   │   └── display.launch.py   # RViz-only launch
│   │   ├── world/
│   │   │   └── ware2.sdf           # Warehouse world (SDF)
│   │   ├── config/
│   │   │   ├── new.rviz            # RViz configuration
│   │   │   └── ros_gz_bridge_gazebo_2.yaml  # Bridge config
│   │   └── meshes/                 # Custom STL meshes (wheels, body, LiDAR)
│   │       ├── base_link.stl
│   │       ├── left_front.stl
│   │       ├── right_front.stl
│   │       ├── left_back.stl
│   │       ├── rigth_back.stl
│   │       └── lidar.stl
│   │
│   └── mecanum_odom/               # Odometry computation package
│       └── mecanum_odom/
│           └── odom_node.py        # Mecanum forward kinematics odometry node
│
├── my_map.yaml                     # Generated map metadata
├── my_map.pgm                      # Generated 2D occupancy grid map
└── media/                          # Simulation demo videos
```

---

## 🤖 How the Robot Works

### Mecanum Wheel Odometry
The robot uses **4 mecanum wheels** — a special type of wheel with angled rollers that allows movement in any direction (forward, sideways, diagonal) without rotating the robot body.

#### Why a Custom Odometry Node?
Gazebo's built-in odometry plugin relies on the **collision geometry** of the wheels for contact and motion calculation. Since our mecanum wheels use a **sphere as the collision primitive** (a common simplification in simulation to approximate the angled roller contact point), Gazebo's default odometry output is inaccurate — the sphere collision doesn't correctly represent how a mecanum wheel actually moves.

To fix this, we wrote a **custom odometry node** (`odom_node.py`) that bypasses Gazebo's plugin entirely and computes odometry directly from **joint states** (actual wheel rotation angles) using mecanum forward kinematics:

```
vx  = r/4 * ( w_lf + w_rf + w_lb + w_rb )   ← forward/backward
vy  = r/4 * (-w_lf + w_rf + w_lb - w_rb )   ← left/right strafe
wz  = r / (4*(L+W)) * (-w_lf + w_rf - w_lb + w_rb)  ← rotation
```

Where `r` = wheel radius, `L` = half wheelbase, `W` = half wheel separation.

This gives accurate real-time position and heading that is fed into SLAM.

### Launch Sequence (Timed)
The launch file carefully sequences everything:

```
0s  → Gazebo starts with warehouse world
0s  → Robot State Publisher starts
5s  → Robot spawns in Gazebo
6s  → ROS-Gazebo bridge starts (connects topics)
7s  → Mecanum odometry node starts
9s  → RViz2 opens for visualization
```

---

## 🚀 How to Run

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- `slam_toolbox` package

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-* ros-jazzy-ros-gz-bridge
```

### Steps

**1. Clone the repository**
```bash
git clone https://github.com/nandanaa555/warehouse-robot-simulation.git
cd warehouse-robot-simulation
```

**2. Build the workspace**
```bash
colcon build
source install/setup.bash
```

**3. Launch the simulation**
```bash
ros2 launch robo_desc gazebo.launch.py
```

**4. Start SLAM for mapping (in a new terminal)**
```bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**5. Teleoperate the robot to map the warehouse**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**6. Save the map**
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

---

## 🗺️ Generated Map

The robot successfully mapped the entire warehouse environment. The output map (`my_map.pgm`) is a 2D occupancy grid:

- **Resolution:** 0.05 m/pixel (5 cm per pixel)
- **Format:** PGM (grayscale) + YAML metadata
- **White** = free space, **Black** = walls/obstacles, **Grey** = unknown

---

## 🎬 Demo Video

Full simulation walkthrough — robot spawning in the warehouse environment, SLAM mapping in progress, and the final generated map in RViz:

📹 **[`Screencast from 2026-06-26 18-34-38.webm`](./media/Screencast%20from%202026-06-26%2018-34-38.webm)**

---

## 👩‍💻 Author

**Nandanaa M S**
- GitHub: [@nandanaa555](https://github.com/nandanaa555)
- Email: nandanaams555@gmail.com
- Bengaluru, Karnataka

---

## 📄 License

MIT License — feel free to use and build on this project!

---

*Built for Rookies Hackathon 2026 — June 28 | Karthikesh Robotics*
