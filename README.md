# 🤖 mobile_robot_ws

A ROS 2 (Humble) workspace designed for simulating a mobile robot navigating through a maze using Gazebo for physics simulation, SLAM Toolbox for mapping and localization, RViz for real-time visualization, and an autonomous exploration node.

---

## 🗂️ Workspace Structure

This workspace includes the following packages:

- 📦 **robot_description/** – URDF/SDF models and launch file to spawn the Pioneer robot.
- 🌍 **robot_world/** – Maze world definition and Gazebo launch files.
- 👁️ **robot_rviz/** – RViz configuration for visualization of sensors, TF, and the robot.
- 🧭 **robot_slam/** – SLAM Toolbox integration for mapping and localization.
- 🚀 **autonomous_explorer/** – Wall-following exploration node (right-hand rule) using LaserScan and Twist.

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

📝 **Note**\
To perform SLAM with autonomous exploration, you can run the full simulation (maze + robot), then launch RViz, start SLAM Toolbox, and finally run the `autonomous_explorer` node:

```bash
# Terminal 1
ros2 launch robot_world maze_with_pioneer.launch.py

# Terminal 2
ros2 launch robot_rviz rviz.launch.py

# Terminal 3
ros2 launch robot_slam slam_launch.py

# Terminal 4
ros2 run autonomous_explorer explorer_node
```

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