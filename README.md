# PX4 Ouster Navigation

ROS2 Humble navigation package for PX4 SITL, Gazebo Classic, Ouster LiDAR, SLAM Toolbox, OctoMap, and 3D A* path planning.

## Tested Environment

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Classic
PX4-Autopilot release/1.16
MAVROS2
AWS RoboMaker Small Warehouse World
```

---

## 1. Run SLAM

### 1.1 Install Dependencies

```bash
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-octomap-server \
  ros-humble-octomap-rviz-plugins \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  ros-humble-sensor-msgs-py \
  ros-humble-aws-robomaker-small-warehouse-world
```

```bash
SCRIPT=$(find /opt/ros/humble -name install_geographiclib_datasets.sh -print -quit)

if [ -n "$SCRIPT" ]; then
  sudo bash "$SCRIPT"
fi
```

---

### 1.2 Clone the PX4 Fork

```bash
git clone --recursive -b ros2-humble-sensored-m100-the \
  https://github.com/siwon0305/PX4-Autopilot.git \
  ~/PX4-Autopilot
```

```bash
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
git submodule update --init --recursive
```

---

### 1.3 Clone the Gazebo Model Fork

```bash
mkdir -p ~/px_ws/src
cd ~/px_ws/src
```

```bash
git clone -b ros2-humble-sensored-m100-the \
  https://github.com/siwon0305/Exploration-PX4-Gazebo.git
```

---

### 1.4 Clone and Build the ROS2 Navigation Package

```bash
mkdir -p ~/drone_ros2_ws/src
cd ~/drone_ros2_ws/src
```

```bash
git clone https://github.com/siwon0305/px4-ouster-nav.git
```

```bash
cd ~/drone_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_mavros_nav
source install/setup.bash
```

---

### 1.5 Run PX4 + Gazebo

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

```bash
export PX4_ROOT=$HOME/PX4-Autopilot
export EXP_ROOT=$HOME/px_ws/src/Exploration-PX4-Gazebo

export PX4_GZ=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export EXP_MODELS=$EXP_ROOT/FUEL/fuel_planner/exploration_manager/models

export AWS_WAREHOUSE_SHARE=$(ros2 pkg prefix aws_robomaker_small_warehouse_world)/share/aws_robomaker_small_warehouse_world

export GAZEBO_MODEL_PATH=$AWS_WAREHOUSE_SHARE/models:$EXP_MODELS:$PX4_GZ/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=$AWS_WAREHOUSE_SHARE:$GAZEBO_RESOURCE_PATH

export PX4_SITL_WORLD=$AWS_WAREHOUSE_SHARE/worlds/small_warehouse/small_warehouse.world
```

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_sensored_m100_the
```

Keep this terminal open after PX4 SITL starts.

---

### 1.6 Run MAVROS

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

---

### 1.7 Takeoff Before Starting SLAM

Before starting SLAM, take off the vehicle first.

In the PX4 SITL console in Terminal 1, run:

```bash
commander takeoff
```

Wait until the vehicle is airborne and stable. Then start SLAM.

---

### 1.8 Run SLAM

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ros2_ws/install/setup.bash
```

```bash
ros2 launch px4_mavros_nav slam_mapping.launch.py
```

---

## 2. Moving the Vehicle During SLAM

There are two ways to move the vehicle while SLAM is running:

```text
Method A: Manual keyboard teleoperation
Method B: Automatic movement using RViz goal pose and 3D A*
```

---

### 2.1 Method A: Manual SLAM with Keyboard Teleoperation

#### Terminal 4: Run cmd_vel Offboard Control

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ros2_ws/install/setup.bash
```

```bash
ros2 run px4_mavros_nav cmd_vel_offboard \
  --ros-args \
  -p use_sim_time:=true \
  -p max_xy_speed:=0.60 \
  -p max_z_speed:=0.25 \
  -p max_yaw_rate:=0.40 \
  -p cmd_timeout:=0.5
```

#### Terminal 5: Run Keyboard Teleoperation

```bash
source /opt/ros/humble/setup.bash
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### 2.2 Method B: SLAM with 3D A* Goal Poses

#### Terminal 6: Run the Live OctoMap Server

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ros2_ws/install/setup.bash
```

```bash
ros2 run octomap_server octomap_server_node \
  --ros-args \
  -r cloud_in:=/ouster/points \
  -p frame_id:=map \
  -p resolution:=0.1 \
  -p use_sim_time:=true
```

#### Terminal 7: Run A* Execute Mode

```bash
ros2 run px4_mavros_nav astar_goal_follower \
  --ros-args \
  -p use_sim_time:=true \
  -p execute:=true \
  -p goal_z:=1.5
```

#### Set a Goal Pose in RViz

In RViz, select `2D Goal Pose` and click the target location.

If `2D Goal Pose` is not visible in the RViz panel or toolbar, use the shortcut:

```text
Ctrl + G
```

Then click the desired target location.  

---

## 3. Save Maps

### 3.1 Save the SLAM Toolbox Map

Run this while SLAM is active.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "<SLAM_MAP_PREFIX>" \
  --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true
```

---

### 3.2 Generate OctoMap

Run this while PX4, Gazebo, MAVROS, and SLAM are active.

If the live OctoMap server is already running from `2.2 Method B`, you do not need to start another one.

Terminal 6:

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ros2_ws/install/setup.bash
```

```bash
ros2 run octomap_server octomap_server_node \
  --ros-args \
  -r cloud_in:=/ouster/points \
  -p frame_id:=map \
  -p resolution:=0.1 \
  -p use_sim_time:=true
```

---

### 3.3 Save OctoMap

Run this in another terminal.

```bash
ros2 run octomap_server octomap_saver_node \
  --ros-args \
  -p octomap_path:="<OCTOMAP_BT_PATH.bt>"
```

---

## 4. Run 3D A* with a Saved OctoMap

This mode runs 3D A* using a previously saved `.bt` OctoMap file.

Run this while PX4, Gazebo, and MAVROS are active.

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ros2_ws/install/setup.bash
```

```bash
ros2 launch px4_mavros_nav octomap_3d_astar.launch.py \
  octomap_path:="<OCTOMAP_BT_PATH.bt>" \
  execute:=true \
  goal_z:=1.5
```

---
