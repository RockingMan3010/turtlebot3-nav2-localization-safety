# System architecture

## High-level ROS 2 data flow

~~~mermaid
flowchart TD
  G["Gazebo Classic 11 | TurtleBot3 Waffle physics"]
  S["LiDAR /scan, wheel odometry /odom, IMU /imu"]
  A["AMCL global map localization"]
  E["Optional EKF local odometry fusion (B only)"]
  N["Nav2: planner, costmaps, controller and velocity smoother"]
  C["Nav2 Collision Monitor | StopFront front polygon"]
  D["Differential-drive plugin | /cmd_vel"]
  G --> S
  S --> A
  S --> E
  A --> N
  E --> N
  S --> N
  S --> C
  N -->|"/cmd_vel_pre_collision"| C
  C -->|"/cmd_vel"| D
  D --> G
~~~

The diagram is conceptual. Launch variants and physical-stop validation are **not** always active together in the same experiment. In the stop-zone test, the monitor input is supplied by a controlled 0.12 m/s test script rather than a Nav2 action.

## Frame ownership is part of the experiment

~~~text
map  ──[AMCL]──>  odom  ──[Gazebo in A | EKF in B]──>  base_footprint
                                                           │
                                                        base_scan
~~~

- **map → odom:** AMCL performs known-map global localization for A **and** B.
- **odom → base_footprint:** Gazebo's differential-drive model in A; robot_localization EKF in B.
- B uses a separately copied TurtleBot3 SDF with Gazebo's odometry TF publication disabled. A baseline model and original installed ROS files are not modified.
- If both Gazebo and EKF publish the same odom TF, transforms compete; preventing duplicate TF ownership is an explicit design requirement.

## Key topics

| Topic | Meaning |
|---|---|
| <code>/scan</code> | Real Gazebo simulated 2D LiDAR observations |
| <code>/odom</code> | Unfiltered odometry |
| <code>/imu</code> | IMU motion and orientation measurements |
| <code>/odometry/filtered</code> | EKF output in B |
| <code>/amcl_pose</code> | AMCL estimated global pose |
| <code>/plan</code> | Nav2 published global path |
| <code>/tf</code>, <code>/tf_static</code> | Coordinate transformations |
| <code>/cmd_vel_pre_collision</code> | Input velocity to stop-zone monitor |
| <code>/cmd_vel</code> | Post-monitor velocity driving Gazebo physics |
| <code>/task2/ground_truth</code> | Independent P3D simulated-physics pose, instrumented runs |

## Collision Monitor implementation

[Configuration](../config/phase5_collision_monitor_stop.yaml) defines the StopFront polygon with forward x from 0.10 m to 0.50 m and lateral y from -0.28 m to +0.28 m, with sensor source <code>/scan</code>. The monitor's output is passed to the Gazebo drive plugin.

In the integrated safety trial, *input positive while output zero* plus *physical pose held stable* is stronger evidence than merely asserting that a stop message appeared.

## Source entrypoints

- [Baseline A launch](../launch/task2_amcl_baseline.launch.py)
- [EKF-assisted B launch](../launch/task2_ekf_assisted.launch.py)
- [Integrated safety launch](../launch/phase5_collision_monitor_integrated.launch.py)
- [Gazebo stop-and-resume script](../scripts/phase7_gazebo_gui_demo_trial01.py)
- [Autonomous Nav2 GUI demo](../scripts/phase7_rviz_nav_demo_trial01.py)
