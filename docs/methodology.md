# Experimental methodology and limitations

## Questions

1. Does filtered wheel odometry improve descriptive Nav2 outcomes relative to direct Gazebo odometry when AMCL remains the common global localizer?
2. Does Nav2 reach its physical simulated goal with dynamically inserted obstacles, and do action status and physical pose agree?
3. Can a separately configured LiDAR stop zone suppress the commanded velocity and stop the robot in *actual Gazebo physics*, then permit motion after the obstacle is removed?

## A/B conditions

- A and B use the same TurtleBot3 Waffle environment, predefined occupancy map, target-goal family and Nav2 navigation concept.
- A uses Gazebo odom TF; B replaces that TF owner with robot_localization EKF fusing <code>/odom</code> and <code>/imu</code>.
- AMCL provides <code>map â†’ odom</code> in *both* A and B.
- Three matched navigation actions per condition are the primary small-sample action comparison.
- Statistical significance or EKF superiority cannot be inferred from n=3.

### Failure avoidance

A copied B model disables the Gazebo odometry TF to avoid conflicting transform broadcasters. Time and coordinate frames must be defined before interpreting pose error. Only use one live simulator/Nav2 stack at a time.

## Dynamic-obstacle protocol

A global plan is requested from Nav2, then an SDF obstacle is spawned in Gazebo during navigation. The procedure logs:

- The initially requested plan and robot pose
- Nav2 action terminal status and elapsed time
- Obstacle insertion result and geometry
- Final simulated Gazebo position/pitch
- Costmap, scan, command and plan publications in the recorded rosbag

A descriptive *physical sanity* check was used for final goal consistency: unadjusted Gazebo world-XY distance to the nominal goal below 0.16 m, with pitch below 0.12 rad. This is **not** an independently calibrated map-frame localization error metric.

Trials 1 and 2 were retained as *negative/ambiguous* evidence. Trials 3â€“5 used the offset obstacle configuration. Plans published after insertion are evidence of planner activity, **not** proof of one distinct full replanning event per message.

## Ground-truth methodology

Project-local model variants add Gazebo P3D physics pose output to <code>/task2/ground_truth</code>. Nine static cylindrical landmark boundaries were matched against occupancy-map cells to obtain a local simulated world-to-map transform. AMCL header timestamps were aligned against interpolated P3D pose samples.

Map resolution: 0.05 m. Reported XY RMSE uses five valid matched pairs in A and six in B, one instrumented run each. Local registration uncertainty, map aliasing, geometry model accuracy and timestamp semantics can dominate centimeter-scale numbers. No globally surveyed truth or real hardware calibration was used.

## Collision Monitor: evidence hierarchy

Three distinct claims must not be conflated:

1. **Isolated software test:** synthetic LaserScan and Twist messages show the Nav2 monitor's pass/stop/resume logic.
2. **Integrated simulated physical test:** real Gazebo LaserScan, actual Nav2 Collision Monitor and Gazebo P3D show blocked output velocity and stopped simulated motion while *controlled* test commands arrive.
3. **Autonomous Nav2 goal stop/replan:** planner-generated velocity and one continuous goal action with stop/replan/recovery would be a different experiment. This repository **does not claim** that complete integration test.

For the GUI physical test, a 0.12 m/s command was injected at <code>/cmd_vel_pre_collision</code>; the monitor set <code>/cmd_vel</code> to zero when front polygon points exceeded the configured threshold. The stop/hold/resume trial finished successfully; the 47-second final montage combines the **separate** RViz Nav2 action footage and Gazebo safety footage.

## Evidence integrity

- Reported CSV and JSON are curated exports from existing experiment reports.
- Raw rosbag DB files, host-specific logs and full academic archives are intentionally **not** publicly uploaded; they remain available in the original private project.
- An earlier integrated headless runner had a *post-stage Python integer-to-float Twist assignment exception* after the meaningful phases. The original failure is preserved privately; independently decoded bag data confirmed physics, and a subsequent GUI runner exited PASS.
- Offline tests validate published data and syntax, **not** ROS/Gazebo integration.

## Open engineering questions

- Stationary-state localization covariance and stability measured over repeated sessions
- Fully aligned trajectory-to-plan path deviation analysis under defined replan epochs
- Multiple environment seeds and confidence intervals
- Single continuous autonomous-goal collision gating, replanning, and recovery
- Migration to supported Gazebo releases and hardware validation with a defined safety case

No fabricated metrics are substituted for missing measurements.
