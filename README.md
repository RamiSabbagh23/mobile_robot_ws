
# 🤖 mobile_robot_ws

A ROS 2 (Humble) workspace designed for simulating a mobile robot navigating through a maze using Gazebo for physics simulation, SLAM Toolbox for mapping and localization, RViz for real-time visualization, and an autonomous exploration node.

---
https://github.com/RamiSabbagh23/mobile_robot_ws/releases/tag/v1.0-results
## 🎥 Results Video

The following video demonstrates the mobile robot simulation results, including Gazebo simulation, maze navigation, SLAM/localization, A* path planning, and Pure Pursuit trajectory tracking.

[Watch / download the results video](https://github.com/RamiSabbagh23/mobile_robot_ws/releases/tag/v1.0-results)

---

## 🗂️ Workspace Structure

This workspace includes the following packages:

- 📦 **robot_description/** – URDF/SDF models and launch file to spawn the Pioneer robot.
- 🌍 **robot_world/** – Maze world definition and Gazebo launch files.
- 👁️ **robot_rviz/** – RViz configuration for visualization of sensors, TF, and the robot.
- 🧭 **robot_slam/** – SLAM Toolbox integration for mapping and localization.
- 🚀 **autonomous_explorer/** – Wall-following exploration node (right-hand rule) using LaserScan and Twist.
- 📍 **astar_plan.py + pure_pursuit_follow.py** – Path planning using A* and trajectory following using a dynamic Pure Pursuit controller with RViz visualization.

---

## 🚀 Getting Started

### 🔧 Build the Workspace

```bash
cd ~/mobile_robot_ws
colcon build
source install/setup.bash
```

---

### ▶️ Launch Commands

#### 🤖 Spawn Robot Only

```bash
ros2 launch robot_description spawn_pioneer.launch.py
```

#### 🧱 Spawn Maze Only

```bash
ros2 launch robot_world maze.launch.py
```

#### 🧩 Launch Robot + Maze (Full Simulation)

```bash
ros2 launch robot_world maze_with_pioneer.launch.py
```

#### 🖼️ Open RViz Visualization

```bash
ros2 launch robot_rviz rviz.launch.py
```

#### 🗺️ Run SLAM Toolbox

```bash
ros2 launch robot_slam slam_launch.py
```

#### 🎮 Control Robot with Keyboard (teleop)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 🤖 Run Autonomous Explorer

```bash
ros2 run autonomous_explorer explorer_node
```

---

## 🧭 Full Procedure: Mapping, Localization, Planning and Navigation

1. **Start Simulation with Mapping**
   Open these in 4 terminals:

```bash
# Terminal 1: Maze + Robot
ros2 launch robot_world maze_with_pioneer.launch.py

# Terminal 2: RViz
ros2 launch robot_rviz rviz.launch.py

# Terminal 3: SLAM Toolbox
ros2 launch robot_slam slam_launch.py

# Terminal 4: Explorer Node
ros2 run autonomous_explorer explorer_node
```

✅ When mapping is complete, a map will be saved in the `map/` folder.

2. **Close all terminals except the one with the Maze (Terminal 1)**

3. **Start Localization, Planning and Navigation**

```bash
# Terminal 2: Localization
ros2 launch robot_localization localization.launch.py
```

🟡 **Important:** After RViz opens, click the "2D Pose Estimate" tool and set the initial pose of the robot in the map.

Then:

```bash
# Terminal 3: A* Planning
ros2 run autonomous_explorer astar_plan --ros-args -p goal_x:=0.0 -p goal_y:=0.0

# Terminal 4: Pure Pursuit Follower
ros2 run autonomous_explorer pure_pursuit_follow
```

This sequence reuses the saved map for localization and executes navigation to the goal.

---

## 🧠 Path Planning and Control (A* + Pure Pursuit)

### 🧭 A* Planner

```bash
ros2 run autonomous_explorer astar_plan --ros-args -p goal_x:=-10.0 -p goal_y:=0.0
```

- Reads map from: `src/autonomous_explorer/map/my_map1.yaml`
- Saves to: `path/my_route.csv` and `my_route.yaml`

### 🚗 Pure Pursuit Follower

```bash
ros2 run autonomous_explorer pure_pursuit_follow
```

- Requires a path file (auto-loads `my_route.csv` or `my_route.yaml`)
- Publishes dynamic RViz topics: `/pp_path_full`, `/pp_path_remaining`, `/pp_goal`, `/pp_lookahead`, etc.
- Includes ALIGN mode: robot turns to face initial heading before tracking

---

## 🔄 Repository Update

To update your local repository to match the remote exactly:

```bash
cd ~/mobile_robot_ws

# (optional) keep a backup of your current state
git switch -c backup-$(date -u +%Y%m%d-%H%M%S)

# fetch everything and prune deleted branches
git fetch origin --prune --tags

# detect the remote's default branch (e.g., main)
DEFBR=$(git symbolic-ref --short refs/remotes/origin/HEAD | sed 's@^origin/@@')
[ -z "$DEFBR" ] && DEFBR=main

# make local match the remote default branch exactly
git checkout "$DEFBR"
git reset --hard "origin/$DEFBR"
git clean -fdx   # removes untracked files/dirs (including build/, .vscode/, etc.)

# submodules (if any)
git submodule sync --recursive
git submodule update --init --recursive --force
```

---


## 📤 Repository Upload

After making changes, you can check and push to the remote:

```bash
git status
git add .
git commit -m "update message here"
git push origin
```

---


## 📦 Required ROS 2 Packages

Before using this workspace, make sure the following packages are installed in your ROS 2 Humble environment:

### 🔧 Core and Common Tools

```bash
sudo apt update && sudo apt install -y \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs
```

### 🧭 SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

### 🌍 Gazebo (Classic) + ROS Integration

```bash
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control
```

### 🧭 Localization (AMCL from Nav2)

```bash
sudo apt install ros-humble-nav2-amcl ros-humble-nav2-map-server
```

### 🎮 Teleop (optional)

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Make sure Gazebo Classic is correctly installed (comes with `ros-humble-desktop`) and sourced:

```bash
sudo apt install ros-humble-desktop
```
