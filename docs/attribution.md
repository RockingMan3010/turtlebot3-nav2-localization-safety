# Authorship, upstream dependencies and asset provenance

## Original implementation

This repository was curated as a public engineering portfolio for **Dhruv Mistry**, who states that the project implementation was carried out independently. It was undertaken in the academic context of **Advanced Methods in Robotics, Group E**, M.Sc. Intelligent Robotics at TH Deggendorf.

The repository's experiment orchestration, configuration differences, analysis and evidence selection should not be confused with ownership of the underlying ROS 2, Nav2, Gazebo or TurtleBot3 packages.

## Third-party code and models

- **ROBOTIS TurtleBot3 and TurtleBot3 simulations**: robot geometry, package resources, and simulation model assets. The SDFs in <code>models/waffle_*</code> are adapted copies of TurtleBot3 Waffle model geometry and plugin configuration. Upstream: https://github.com/ROBOTIS-GIT/turtlebot3_simulations (Apache License 2.0).
- **ROS 2 Humble and Navigation2**: external runtime, Nav2 planner/controller, Collision Monitor, AMCL, ROS message definitions and lifecycle integrations. Upstream: https://docs.ros.org/en/humble/ and https://navigation.ros.org/.
- **Gazebo Classic**: simulator, physics and sensors. Upstream: https://classic.gazebosim.org/.
- **robot_localization**: EKF filtering package. Upstream: https://github.com/cra-ros-pkg/robot_localization.
- **RViz2**: visualization software, not the simulator.

This repository does not redistribute Ubuntu package binaries, downloaded <code>.deb</code> packages, external headers, copied system libraries or third-party installed ROS package trees.

### License scope

The Apache License 2.0 file included in this repository governs the original code/doc contributions by the repository author to the extent he owns them. Adapted upstream ROBOTIS models are **not relicensed** and retain upstream attribution, copyright and applicable Apache-2.0 terms. Other ROS packages remain separately licensed upstream.

## Evidence provenance

All figures and the included 47-second MP4 were captured or generated from this project's simulator runs. The MP4 combines separate genuine Gazebo and RViz demonstrations, not a single autonomous navigation-plus-monitor action.

Public data tables are curated from existing reports. Historical failed attempts, machine usernames/paths, raw system logs, full rosbags, private academic records and installation binaries were excluded from the GitHub repository.

## Scientific presentation

A polished project is not an excuse to invent precision: published RMSE is locally calibrated and limited-sample, and published stopping is **Gazebo simulation**, not a physical emergency-stop guarantee.
