# Technical interview and viva Q&A

These answers describe the actual experiments, not hypothetical capabilities.

### 1. What was built?
A TurtleBot3 Waffle Nav2 simulation with known-map AMCL localization, a controlled comparison of direct versus EKF-fused odometry, dynamic-obstacle navigation trials and a separately instrumented LiDAR-based stop-and-resume safety layer.

### 2. Gazebo versus RViz?
Gazebo simulates robot physics, model state and sensors. RViz visualizes ROS topics, transforms, map and planned paths. Gazebo was the simulator; RViz was the diagnostic/visualization interface.

### 3. Why use AMCL?
AMCL uses particle filtering with a known map, LiDAR observations and motion information to estimate a global map-frame robot pose.

### 4. Why use EKF if AMCL is already running?
They solve different problems. AMCL estimates global map location; EKF fuses wheel odometry and IMU for smoother local odometry. EKF did not replace AMCL.

### 5. What actually changed between A and B?
The odometry source consumed by Nav2 and the owner of the odom-to-robot TF. In B, robot_localization EKF published this TF and used odometry plus IMU. The AMCL map-to-odom localizer was unchanged conceptually.

### 6. Which configuration performed better?
Both completed three of three actions. B's mean action time was 4.104 s versus 4.347 s for A. These small samples do not support a statistical superiority claim.

### 7. Why should only one node publish a TF?
Two competing odom-to-base transformations can create inconsistent frame estimates and make navigation unstable. Gazebo TF publication was disabled in B's copied robot model.

### 8. What is ground truth here?
The simulator's independent P3D physics-based robot pose. It was timestamp-aligned and locally registered to the occupancy map before comparing with AMCL.

### 9. What does RMSE tell us?
The root mean square of pose-position discrepancies against the chosen reference after time/frame alignment. Because the map fit had uncertainty, the measured ~1.5 cm and ~1.4 cm errors are exploratory, not certified absolute accuracy.

### 10. How were obstacles tested?
SDF box obstacles were spawned while Nav2 was navigating. We inspected action outcomes, plan/costmap evidence and actual Gazebo final robot poses.

### 11. Why did Trial 2 count as inconsistent despite a successful Nav2 action?
The action returned SUCCEEDED, but the robot was approximately 0.85 m from the intended goal in the unadjusted simulated-world sanity comparison. Action status alone was insufficient to claim that the physical simulation achieved the goal.

### 12. What does a costmap do?
A costmap represents obstacles and navigation cost. Nav2 uses local/global costmaps to plan and follow safe trajectories as new sensor measurements arrive.

### 13. Replanning versus stopping?
Replanning finds a revised route toward the destination. A stop-zone Collision Monitor is a separate velocity filter that suppresses movement when nearby LiDAR obstacles violate a configured region.

### 14. How did Collision Monitor detect danger?
Real Gazebo LaserScan data arrived on /scan. A forward polygon covering 0.10–0.50 m ahead was checked. When its point threshold was exceeded, the monitor commanded zero outgoing velocity.

### 15. How did you verify actual stopping?
We correlated nonzero input commands, zero output commands and nearly unchanged independent Gazebo physics pose during the settled obstacle interval, then measured motion after removing the obstacle.

### 16. Was stopping part of a live Nav2 goal?
No. The physical stop test injected velocity directly into the monitor input while Nav2 was active but idle. The Nav2 autonomous goal was demonstrated in a separate authentic run.

### 17. What role did ROS bags serve?
They persisted ROS sensor, odometry, TF, command, planner and physics topics for offline analysis. Raw bags were not published in the curated public repo.

### 18. Did you test a physical TurtleBot3?
No. The reported measurements are from Gazebo simulation only.

### 19. What were the main limitations?
Small A/B samples, short ground-truth alignment windows, locally calibrated map geometry, no independently established full path-deviation/stationary stability metric and no hardware safety validation.

### 20. What would you improve next?
Increase trial counts, quantify covariance/stationary stability and path deviation, integrate the safety gate into one actual Nav2 goal/recovery run and migrate to a maintained Gazebo version.
