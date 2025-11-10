# Autonomous Navigation with 3D LiDAR (ROS Noetic)

Differential-drive robot · Global & local planning · Gazebo simulation

This repository contains a complete navigation stack built for ROS Noetic that demonstrates autonomous navigation using a 3D Velodyne VLP-16 LiDAR. The system uses a pre-built static map and AMCL for localization (no SLAM), converts the 3D point cloud into a 2D laser scan for compatibility with classic ROS planners, and performs global and local planning to navigate indoor environments in Gazebo.

Ideal for learners who want hands-on experience with:
- ROS Navigation Stack (move_base)
- Costmap configuration and inflation tuning
- LiDAR pointcloud processing (3D → 2D)
- Path planning in tight indoor corridors
- Gazebo robot simulation and RViz visualization

## Key Features
- Custom differential-drive robot (URDF + Gazebo plugins)
- 3D Velodyne VLP-16 LiDAR integration
- pointcloud_to_laserscan conversion (3D → 2D)
- AMCL-based localization (uses a prebuilt map)
- A* global planner
- TrajectoryPlannerROS local planner
- Tuned global and local costmaps with separate inflation settings
- Gazebo indoor office world and RViz for visual debugging (global/local plans, costmaps)

## How it works (high level)
1. Prebuilt map  
   - A static .pgm map and associated YAML are served by map_server.

2. Localization (AMCL)  
   - The robot fuses:
     - /scan (generated from the 3D LiDAR via pointcloud_to_laserscan)
     - /map
     - /odom  
   to estimate its pose using AMCL.

3. Global planning (A*)  
   - A* computes long-range routes over the static map.
   - A larger global inflation radius biases the global path toward wide, safer regions.

4. Local planning (TrajectoryPlannerROS)  
   - The local costmap uses a smaller footprint and low inflation to let the robot squeeze through narrow passages while reacting to dynamic obstacles.

5. Obstacle avoidance  
   - Dynamic obstacles detected in /scan are handled by the local planner, which generates on-the-fly detours and collision-free commands.

## Requirements
- Ubuntu (compatible with ROS Noetic)
- ROS Noetic
- Gazebo (compatible version with Noetic)
- Catkin workspace
- Velodyne VLP-16 drivers/tools (if testing on real hardware)

## Installation (quick)
Clone into your catkin workspace and build:

```bash
cd ~/catkin_ws/src
# Option 1: clone this repository
git clone https://github.com/Goshan189/autonomous_wheelchair.git

# Option 2: original upstream (replace YOUR_USERNAME)
# git clone https://github.com/YOUR_USERNAME/robot_3d_lidar.git

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run the simulation
Launch the full navigation demo:

```bash
roslaunch robot_3d_lidar navigation.launch
```

This launch file starts:
- Gazebo and the office world
- The robot URDF and Gazebo plugins
- Velodyne/LiDAR processing and pointcloud_to_laserscan
- map_server with the static map
- AMCL, move_base, and RViz visualization

## Tuning notes (practical tips)
Global costmap
- Use a larger inflation radius (≈ 0.40–0.50 m) so the global planner keeps paths away from risky, tight areas.

Local costmap
- Use a smaller inflation radius (≈ 0.12–0.18 m) so the robot can pass through narrow gaps while the local planner still avoids obstacles.

Planner tuning highlights
- Increase pdist_scale to make the local planner adhere more closely to the global path.
- Reduce occdist_scale to reduce freezing in narrow corridors.
- sim_time ≈ 1.5–2.0 for smoother local trajectory curves.

## Visualization
Open RViz to inspect:
- Global plan
- Local plan
- Costmaps (global & local)
- Laser scans and pointclouds

## Contributing & Notes
- This project is built for learning and experimentation. You can adapt the costmaps, controllers, and planners for different robots or environments.
- If you test with a real VLP-16, ensure drivers and coordinate frames are configured correctly.

License: (add your preferred license or leave as-is)

Questions or improvements? Open an issue or submit a PR.
