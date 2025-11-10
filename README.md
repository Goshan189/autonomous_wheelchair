# Autonomous Navigation With 3D LiDAR in ROS Noetic

Differential Drive Robot | Global + Local Planning | Gazebo Simulation

This project is a full navigation stack built from scratch using ROS Noetic, Gazebo, and a 3D Velodyne LiDAR (VLP-16).
The robot performs global path planning, local obstacle avoidance, and real-time navigation inside a custom indoor world — no SLAM needed.
You provide a pre-built map, the robot localizes using AMCL, and moves with classical ROS planners.

This repo is perfect if you're learning:

ROS Navigation Stack (move_base)

Costmaps & inflation tuning

LiDAR processing (3D → 2D scan)

Path planning through narrow hallways

Gazebo robot simulation

# Features

* Custom differential-drive robot (URDF + Gazebo plugins)

* 3D Velodyne VLP16 LiDAR

* 3D → 2D laser conversion (pointcloud_to_laserscan)

* AMCL localization (no SLAM required)

* A* global planner

* TrajectoryPlannerROS local planner

* Tuned costmaps (global + local inflation)

* Gazebo indoor office world

* RViz visualization (global plan, local plan, costmaps)

# How the System Works
1️⃣ Prebuilt Map (PGM + YAML)

A static .pgm map is loaded using map_server.

2️⃣ Localization – AMCL

The robot uses:

/scan (generated from 3D LiDAR)

/map

/odom

to estimate its pose.

3️⃣ Global Planning – A*

The global planner computes a long-range path over the static map.
Large inflation radius keeps the robot away from tight and risky places.

4️⃣ Local Planning – TrajectoryPlannerROS

The local costmap is much smaller and uses tiny inflation, allowing the robot to squeeze through narrow corridors in real time.

5️⃣ Obstacle Avoidance

Dynamic obstacles appear from /scan.
Local planner decides detours on the fly.

#  Installation
Clone into a Catkin Workspace

cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/robot_3d_lidar.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Run the Simulation

roslaunch robot_3d_lidar navigation.launch

This automatically starts:

Gazebo world

Robot URDF

LiDAR + pointcloud_to_laserscan

Map server

# Key Parameter Tuning
# Global Costmap

Large inflation radius (0.40–0.50 m)

Makes global path stay in wide, safe areas

# Local Costmap

Small inflation radius (0.12–0.18 m)

Lets robot pass narrow gaps without getting stuck

# Planner Tuning Highlights

Increase pdist_scale → stick to global path more

Reduce occdist_scale → avoid freezing in narrow spaces

sim_time = 1.5–2.0 → smoother curves
