# 🤖 mobile_robot_ws

A ROS 2 (Humble) workspace designed for simulating a mobile robot navigating through a maze using Gazebo for physics simulation, SLAM Toolbox for mapping and localization, and RViz for real-time visualization.

---

## 🗂️ Workspace Structure

This workspace includes the following packages:

- 📦 **robot_description/** – URDF/SDF models and launch file to spawn the Pioneer robot.
- 🌍 **robot_world/** – Maze world definition and Gazebo launch files.
- 👁️ **robot_rviz/** – RViz configuration for visualization of sensors, TF, and the robot.
- 🧭 **robot_slam/** – SLAM Toolbox integration for mapping and localization.

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

---

📝 **Note**  
To perform SLAM and generate a map of the maze, be sure to run the full simulation, open RViz, and start the SLAM Toolbox concurrently:

```bash
# Terminal 1
ros2 launch robot_world maze_with_pioneer.launch.py

# Terminal 2
ros2 launch robot_rviz rviz.launch.py

# Terminal 3
ros2 launch robot_slam slam_launch.py
```