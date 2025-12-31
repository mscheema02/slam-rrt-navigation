# SLAM + Occupancy Mapping + RRT Path Planning + Waypoint Navigation (ROS2)

[![Demo Video](https://img.youtube.com/vi/8zLRY0uYAzY/0.jpg)](https://www.youtube.com/watch?v=8zLRY0uYAzY)

> If you don’t want a thumbnail, you can just paste your link here:  
> **Demo:** https://www.youtube.com/watch?v=8zLRY0uYAzY

---

## Overview

This project implements an end-to-end indoor navigation pipeline in **ROS2 + Gazebo**:

1. **Create a custom indoor world** from a floorplan using Gazebo Building Editor.
2. **Generate an occupancy map** by exploring the environment with a LiDAR robot (SLAM/mapping).
3. **Post-process the map** to match world scale and **inflate obstacles** to account for robot footprint.
4. **Plan a collision-free path** from start → goal using **RRT (Rapidly-exploring Random Tree)**.
5. **Execute the path** using a **waypoint runner** that repeatedly updates a `drive_to_goal` controller.

---

## Key highlights

- Built a complete **mapping → planning → execution** pipeline in ROS2, integrating simulation, occupancy grids, sampling-based planning (RRT), and waypoint-level control.
- Implemented **map scaling + obstacle inflation** to make plans physically feasible for a robot footprint (safer clearance from walls).
- Automated waypoint execution by programmatically updating goals and waiting for **goal-reached** completion before continuing.

---

## Table of Contents

- [Overview](#overview)
- [Key highlights (resume-friendly)](#key-highlights-resume-friendly)
- [Approach](#approach)
  - [1) World creation](#1-world-creation)
  - [2) SLAM + occupancy map generation](#2-slam--occupancy-map-generation)
  - [3) Map scaling + obstacle inflation](#3-map-scaling--obstacle-inflation)
  - [4) RRT path planning](#4-rrt-path-planning)
  - [5) Waypoint navigation execution](#5-waypoint-navigation-execution)
- [Technologies](#technologies)
- [Repository layout](#repository-layout)
- [How to run](#how-to-run)
  - [0) Prerequisites](#0-prerequisites)
  - [1) Build the ROS2 workspace](#1-build-the-ros2-workspace)
  - [2) Launch the world + robot](#2-launch-the-world--robot)
  - [3) Generate and save the map](#3-generate-and-save-the-map)
  - [4) Run map processing + RRT planning](#4-run-map-processing--rrt-planning)
  - [5) Run waypoint navigation](#5-run-waypoint-navigation)
- [Learnings](#learnings)
- [Resume bullets (copy/paste)](#resume-bullets-copypaste)
- [Key parameters](#key-parameters)
- [Troubleshooting](#troubleshooting)
- [Future improvements](#future-improvements)

---

## Approach

### 1) World creation
- Select a floorplan image and import it as a reference.
- Use Gazebo **Building Editor** to recreate walls/rooms.
- Export the world into a ROS2 package so it can be launched consistently.

**Goal:** Create a realistic indoor environment for mapping + navigation experiments.

### 2) SLAM + occupancy map generation
- Spawn a LiDAR-equipped robot and explore the environment.
- Build an **occupancy grid**:
- The result is an occupancy grid:
  - **free space**
  - **occupied (walls/obstacles)**
  - **unknown**

**Goal:** Convert sensor observations into a grid suitable for planning.

### 3) Map scaling + obstacle inflation
To create safe, feasible plans:
- Resize the map image so its scale matches the Gazebo world coordinates.
- Inflate obstacles (dilation) by approximately the robot radius (plus margin).

**Why inflation matters:** It prevents “corner cutting” and reduces wall collisions by enforcing clearance.

### 4) RRT path planning
- Use RRT to sample points in free space and grow a tree toward the goal (collision-free).
- Stop when a node enters the goal region, then backtrack to recover a path (sample random points in free spce until reaching goal).
- Convert the path into a sequence of waypoints.

**Output example:** A waypoint sequence such as,
```
[(x1,y1), (x2,y2), ..., (xN,yN)]
```

### 5) Waypoint navigation execution
- A `navigation.py`-style script iterates through waypoints.
- For each waypoint it updates the controller goal via ROS2 params:
  - `ros2 param set /drive_to_goal dest "[x, y]"`
- It waits for **Goal reached** before sending the next waypoint.

**Goal:** Turn an offline plan into a repeatable autonomous run.

---

## Technologies

- **VMWare Workstation Pro with Ubuntu 22.04** (running on a **Windows PC**)
- **ROS2 with Gazebo for Simulations**
- **Python 3** (map processing, RRT, waypoint runner)
- Typical ROS2 stack used in this pipeline:
  - `nav_msgs` (occupancy grids / maps)
  - `sensor_msgs` (LiDAR)
  - `geometry_msgs` (Twist commands)
- Common processing libraries:
  - NumPy
  - OpenCV (resize + dilation / inflation)

---

## Repository layout

Suggested structure (adjust to your repo):

```
project3/
  src/
    <your_ros2_pkg>/
      launch/
      worlds/
      scripts/
  maps/
    raw/
    processed/
  planning/
    map_processing.py
    rrt_planner.py
  navigation/
    navigation.py
  README_Project3.md
```

---

## How to run

### 0) Prerequisites
On Ubuntu 22.04 inside VMware:
- ROS2 installed (commonly **Humble** for Ubuntu 22.04)
- Gazebo + ROS2 Gazebo integration installed
- A ROS2 workspace containing your:
  - world launch
  - robot bringup
  - mapping (SLAM) pipeline
  - map processing + RRT planner scripts
  - waypoint runner + controller (`drive_to_goal`)

### 1) Build the ROS2 workspace

```bash
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build --symlink-install
source install/setup.bash
```

### 2) Launch the world + robot

```bash
ros2 launch <your_pkg> <your_world_launch>.launch.py
```


- Gazebo shows your world correctly
- Robot is spawned and publishes to:
  - `/scan` (LiDAR)
  - `/odom`
  - camera topics (if applicable)

### 3) Generate and save the map

Drive the robot around until the map is complete, then save:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/raw/my_map
```

Outputs typically include:
- `my_map.pgm` (or `.png`)
- `my_map.yaml`

> If your project saves maps differently, use that method — the requirement is: map image + metadata.

### 4) Run map processing + RRT planning

#### A) Process the map (scale + inflate obstacles)

```bash
python3 planning/map_processing.py   --input maps/raw/my_map.pgm   --yaml maps/raw/my_map.yaml   --output maps/processed/my_map_inflated.png   --inflate_radius_m 0.20
```

#### B) Run RRT planner

```bash
python3 planning/rrt_planner.py   --map maps/processed/my_map_inflated.png   --start "x0,y0"   --goal "xg,yg"   --output planning/waypoints.json
```

### 5) Run waypoint navigation

Start the controller node if needed:

```bash
ros2 run <your_pkg> drive_to_goal
```

Then run the waypoint runner:

```bash
python3 navigation/navigation.py --waypoints planning/waypoints.json
```

Behavior:
- sets `dest` for each waypoint
- waits until the controller reports **Goal reached**
- proceeds to the next waypoint

---

## Learnings

- **Occupancy grid fundamentals:** understanding map resolution/origin, free vs occupied thresholds, and how grid encoding affects planning.
- **Coordinate frame hygiene:** consistently converting between map/world/robot frames is essential for correct start/goal placement.
- **Collision safety:** obstacle inflation is a practical must-have when planning for non-point robots.
- **Sampling-based planning tradeoffs:** RRT performance depends heavily on step size, sampling space, and goal tolerance (speed vs path quality).
- **Systems debugging:** verifying topic wiring (`/scan`, `/odom`, `/cmd_vel`) and validating each stage independently (map → processed map → path → execution).

---

## Key parameters


Depending on implementation, these are commonly used:

### Controller (`drive_to_goal`)
- `dest`: goal position `[x, y]`
- `max_vel`: max speed cap
- `vel_gain`: velocity scaling (aggressiveness)

### Planner
- `step_size`: RRT expansion step
- `goal_tolerance`: how close is “good enough”
- `max_iters`: limit on tree growth

### Map processing
- `inflate_radius_m`: inflation radius (robot radius + margin)
- scale / resolution: ensure processed map aligns with Gazebo coordinates

---

## Troubleshooting

- **Robot plans a path through walls**
  - Map scaling/resolution is likely mismatched to the Gazebo world.
  - Re-check metadata (resolution + origin) and coordinate conversions.

- **Robot drives too close to walls**
  - Increase obstacle inflation radius.

- **RRT never reaches the goal**
  - Confirm start/goal are in free space.
  - Increase `max_iters`, reduce `step_size`, or increase `goal_tolerance`.

- **Robot doesn’t reach waypoints**
  - Increase `goal_tolerance` slightly or adjust controller `vel_gain`.
  - Confirm the controller interprets `dest` in the intended frame.

---

## Future improvements

- Smooth the RRT path (e.g., shortcutting / spline smoothing) to reduce turns and improve controller tracking.
- Add a cost map (distance-to-obstacle penalty) to bias the planner away from narrow passages.
- Replace waypoint-only control with a local planner (e.g., Nav2) for better obstacle handling and recovery.
