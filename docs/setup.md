# Setup and reproducibility

This is a **curated research/engineering project repository**, not a self-contained apt package, Docker image, or production deployment.

## Validated source environment

- Ubuntu 22.04 LTS, Linux x86_64
- ROS 2 Humble
- Gazebo Classic 11 with ROS–Gazebo integration
- ROBOTIS TurtleBot3 Waffle simulation and navigation packages
- Navigation2 (Nav2), AMCL, and robot_localization EKF
- Python 3.10-era ROS environment
- Optional: RViz2, matplotlib, NumPy, PyYAML, SQLite rosbag utilities

Gazebo Classic reached end-of-life in January 2025. Reproducing this setup is for a historical/course-compatible experiment; consider the maintained Gazebo stack for new deployments.

## External dependencies

The repository assumes the following ROS packages are discoverable from the same sourced ROS 2 installation:

- <code>turtlebot3_gazebo</code>, <code>turtlebot3_navigation2</code>
- <code>gazebo_ros</code>
- <code>nav2_bringup</code>, <code>nav2_collision_monitor</code>
- <code>nav2_simple_commander</code> from Nav2
- <code>robot_localization</code>
- <code>robot_state_publisher</code>
- Standard ROS message/interface packages (geometry, sensor, odometry, Gazebo services)

Official installation references:
- [ROS 2 Humble documentation](https://docs.ros.org/en/humble/)
- [ROBOTIS TurtleBot3 simulations, Humble](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble)
- [Nav2 documentation](https://navigation.ros.org/)

This repository does **not** ship apt/deb binaries or third-party system libraries. The original workstation had a project-local workaround for a missing diagnostic_updater shared library; fresh installations should resolve the system dependency properly. If B's EKF cannot load that library, stop rather than silently dropping the EKF.

## Local checkout

~~~bash
git clone https://github.com/RockingMan3010/turtlebot3-nav2-localization-safety.git
cd turtlebot3-nav2-localization-safety
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
mkdir -p logs evidence
~~~

**Use only one Gazebo/Nav2 stack at a time.** Terminate A before starting B, and never start simultaneous odom-to-base TF publishers.

## Condition A: baseline AMCL + wheel odometry

~~~bash
ros2 launch ./launch/task2_amcl_baseline.launch.py gazebo_gui:=true
~~~

Expected roles:
- Gazebo provides the odom-to-base TF and <code>/odom</code>.
- AMCL provides map-to-odom TF.
- Nav2 uses the known map and robot's estimated pose.

If the robot pose is not initialized, use RViz 2D Pose Estimate or the project script after Gazebo and Nav2 are active:

~~~bash
python3 scripts/initialize_amcl_from_gazebo.py --apply
~~~

## Condition B: EKF-assisted local odometry

Shut down the A stack, then launch:

~~~bash
ros2 launch ./launch/task2_ekf_assisted.launch.py gazebo_gui:=true
~~~

Expected roles:
- B's copied robot SDF disables Gazebo odom TF publication.
- EKF fuses wheel odometry and IMU, publishes filtered odometry and odom TF.
- AMCL remains map-to-odom TF owner.

Before evaluating B, confirm <code>/odometry/filtered</code>, <code>/tf</code>, and diagnostic-updater availability.

## Live simulated stop-region example

~~~bash
ros2 launch ./launch/phase5_collision_monitor_integrated.launch.py gazebo_gui:=true
~~~

In a second shell with the same ROS setup:

~~~bash
mkdir -p logs evidence
python3 scripts/initialize_amcl_from_gazebo.py --apply
python3 scripts/phase7_gazebo_gui_demo_trial01.py
~~~

This demonstration uses Gazebo's own LiDAR and physics and **injects a controlled velocity at the Collision Monitor input**. It does not execute a Nav2 goal at the same time. An actual planner-generated action must be evaluated separately.

## Offline validation: no ROS installation necessary

~~~bash
python3 tools/validate_evidence.py
python3 -m compileall -q scripts launch tools
~~~

These commands validate *curated results consistency and Python syntax*. They do **not** test simulator runtime, timing, hardware safety, or GUI behavior.

## Troubleshooting checklist

1. Confirm all ROS packages resolve, and only one simulated Gazebo world is running.
2. Confirm robot spawn is complete and <code>/scan</code>, <code>/odom</code>, and <code>/tf</code> exist.
3. Check simulation-time settings and initial AMCL pose before sending a goal.
4. For EKF, verify there is **exactly one** odom-to-base TF publisher.
5. For Collision Monitor, confirm the lifecycle state is active and both incoming and outgoing velocity topics are populated.
6. If a navigation action succeeds but robot physics is far from the goal, **do not** call it a physical success; inspect frame alignment and the resulting Gazebo pose.
7. On a headless or overloaded host, disable Gazebo GUI and capture ROS evidence first.

## Scope of reproducibility

The code and configs were used in the original controlled Ubuntu ROS 2 environment. Some Python scripts were converted from fixed workstation paths to repository-relative paths for publication; this curated checkout has **offline syntax/evidence validation**, not a complete clean-machine end-to-end integration re-run. See [Results](results.md).
