# SLAM + Occupancy Mapping + RRT Path Planning + Waypoint Navigation (ROS2)

[![Demo Video](https://img.youtube.com/vi/8zLRY0uYAzY/0.jpg)](https://www.youtube.com/watch?v=8zLRY0uYAzY)

> If you don’t want a thumbnail, you can just paste your link here:  
> **Demo:** https://www.youtube.com/watch?v=8zLRY0uYAzY

---

## Overview

This project builds an end-to-end robotics navigation pipeline in **ROS2 + Gazebo**:

1. **Create a custom indoor world** from a floorplan in Gazebo (Building Editor).
2. **Run SLAM / mapping** using a LiDAR robot to generate an **occupancy map**.
3. **Post-process the map** to match world scale and expand obstacles to account for robot size.
4. **Plan a collision-free path** from start → goal using **RRT (Rapidly-exploring Random Tree)**.
5. **Execute the path** using a **waypoint navigation** script that repeatedly sets the next goal for a `drive_to_goal` controller.

This README focuses on **how to run and reproduce the pipeline**, and what each step is doing.

---

## Table of Contents

- [Overview](#overview)
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
- [Key parameters](#key-parameters)
- [Troubleshooting](#troubleshooting)

---

## Approach

### 1) World creation
- A floorplan image is selected and imported as a reference.
- The Gazebo **Building Editor** is used to recreate walls/rooms.
- The world is exported and placed into a ROS2 package so it can be launched reliably.

**Goal:** Make a realistic indoor layout that the robot can map and navigate.

### 2) SLAM + occupancy map generation
- A LiDAR-equipped robot is spawned in the world.
- The robot is driven around to observe hallways/rooms and build a map.
- The result is an occupancy grid:
  - **free space**
  - **occupied (walls/obstacles)**
  - **unknown**

**Goal:** Convert sensor observations into a usable grid representation for planning.

### 3) Map scaling + obstacle inflation
To safely plan for a real robot footprint:
- The map image is resized so its scale matches the Gazebo world scale.
- Obstacles are **dilated / inflated** by approximately the robot radius (plus margin).

**Why inflation matters:** Without it, the planner can “clip” corners or plan paths that are too close to walls.

### 4) RRT path planning
- RRT is used to search a collision-free path from start to goal.
- The planner samples random points in free space and grows a tree until it reaches the goal region.
- Once a path is found, it’s converted into a waypoint list.

**Output:** A waypoint sequence such as:
```
[(x1,y1), (x2,y2), ..., (xN,yN)]
```

### 5) Waypoint navigation execution
- A `navigation.py`-style script loops through waypoints.
- For each waypoint, it updates the controller’s goal via ROS2 params, e.g.:
  - `ros2 param set /drive_to_goal dest "[x, y]"`
- It waits for the controller to report **“Goal reached”** before moving to the next waypoint.

**Goal:** Turn a planned path into a repeatable, automated run.

---

## Technologies

- **VMWare Workstation Pro with Ubuntu 22.04** (running on a **Windows PC**)
- **ROS2 with Gazebo for Simulations**
- **Python 3** (planning + navigation scripting)
- Typical ROS2 message stack used in this pipeline:
  - `nav_msgs` (occupancy grids / maps)
  - `geometry_msgs` (Twist commands)
  - `sensor_msgs` (LiDAR)
- Image / grid processing commonly uses:
  - NumPy
  - OpenCV (for dilation / resizing)

> Note: Exact package names and scripts depend on your repo structure (this README gives standard commands and patterns).

---

## Repository layout

This is a **suggested** structure (adjust to match your repo):

```
project3/
  src/
    <your_ros2_pkg>/
      launch/
      worlds/
      scripts/
      <nodes>
  maps/
    raw/
    processed/
  planning/
    rrt_planner.py
    map_processing.py
  navigation/
    navigation.py
  README.md
```

---

## How to run

### 0) Prerequisites
On Ubuntu 22.04 inside VMware:
- ROS2 installed (commonly **Humble** on Ubuntu 22.04)
- Gazebo + ROS2 Gazebo integration installed
- A working workspace containing:
  - the world launch
  - robot bringup
  - SLAM/mapping setup (or equivalent mapping pipeline)
  - your RRT + navigation scripts

### 1) Build the ROS2 workspace

```bash
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build --symlink-install
source install/setup.bash
```

### 2) Launch the world + robot

Launch the Gazebo world and spawn the robot:

```bash
ros2 launch <your_pkg> <your_world_launch>.launch.py
```

Verify:
- Gazebo shows your world correctly
- Robot is spawned and publishes:
  - `/scan` (LiDAR)
  - `/odom`
  - camera topics (if applicable)

### 3) Generate and save the map

Run your mapping + drive around the environment until the map is complete.

If you are using a standard map server workflow, saving might look like:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/raw/my_map
```

This typically outputs:
- `my_map.pgm` (or `.png`)
- `my_map.yaml`

> If your project uses a custom method of saving the map, use that instead — the key is to export a map image + metadata.

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

Start your `drive_to_goal` controller node (if it’s not already running):

```bash
ros2 run <your_pkg> drive_to_goal
```

Then run the waypoint runner:

```bash
python3 navigation/navigation.py --waypoints planning/waypoints.json
```

Typical behavior:
- Sets `dest` for each waypoint
- Waits until the controller logs/returns **Goal reached**
- Proceeds to the next waypoint automatically

---

## Key parameters

Depending on your implementation, these are commonly used:

### `drive_to_goal` controller
- `dest`: goal position `[x, y]`
- `max_vel`: max speed cap
- `vel_gain`: velocity scaling (aggressiveness)

### Planner
- `step_size`: RRT expansion step
- `goal_tolerance`: how close is “good enough”
- `max_iters`: safety cap on tree growth

### Map processing
- `inflate_radius_m`: obstacle inflation radius (robot radius + safety margin)
- `scale_factor` / map resolution: ensure map aligns with Gazebo coordinates

---

## Troubleshooting

- **Robot plans a path through walls**
  - Your map scaling/resolution is mismatched with the Gazebo world scale.
  - Re-check map metadata (resolution + origin) and your coordinate conversions.

- **Robot drives too close to walls**
  - Increase obstacle inflation radius slightly.

- **RRT never reaches the goal**
  - Increase `max_iters`, reduce `step_size`, or confirm start/goal are in free space.
  - Verify that the map image has correct free/occupied encoding.

- **Waypoints look correct but robot doesn’t reach them**
  - Increase `goal_tolerance` or adjust controller `vel_gain`.
  - Confirm the frame conventions (map/world vs odom) and that `dest` is interpreted in the correct frame.

---

## Where to add your YouTube link

At the very top of this README, replace `VIDEO_ID` with your YouTube ID (the part after `v=` in a YouTube URL).

Example:
- URL: `https://www.youtube.com/watch?v=dQw4w9WgXcQ`
- VIDEO_ID = `dQw4w9WgXcQ`
