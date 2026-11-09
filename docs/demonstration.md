# Genuine GUI demonstration and how to read it

[Watch the 47-second video](../assets/demo.mp4).

This is a **stitched presentation clip of two separately recorded authentic
Gazebo/RViz runs**. It is not a simulated animation generated from a slide deck.

## Sequence

| Segment | Duration | What appears | What is verified |
|---|---:|---|---|
| Title | ~3 s | Project identity | Presentation label |
| RViz | ~17 s | Real ROS visualization of Nav2 movement | Autonomous action returned SUCCEEDED |
| Gazebo | ~21 s | Real Gazebo Waffle and front LiDAR stop/hold/resume | Monitor stopped simulated physics |
| Summary | ~6 s | Numbers and measured trace | Matches exported evidence |

Video timing is approximate by segment, with the complete MP4 encoded as H.264,
1280×720, 12 fps, 47 seconds total.

## Distinguish the experiments

**Autonomous navigation footage:** Nav2 computed velocity as the robot moved
toward the selected map goal. The action returned SUCCEEDED; the unadjusted
simulation-world goal separation in that separate GUI run was about 0.112 m.
This was a demonstration, not a new independent replicate in the 3+3 A/B study.

**Collision Monitor footage:** The Nav2 stack was active but no navigation
goal was in progress. A control script directly supplied +0.12 m/s input to
the Collision Monitor; the real simulated Gazebo LaserScan detected an
obstacle and the monitor suppressed output velocity. When the obstacle was
deleted, the same input resumed the simulated robot's movement.

There is **no claim that an ongoing Nav2 goal was automatically paused and
replanned during the monitor demonstration**. A continuous end-to-end
autonomous navigation safety run is possible future work.

## Why raw rosbags are not in this repo

The public repo includes selected measured figures, CSV and JSON summaries.
Large raw rosbags and full workstation logs were excluded to keep it small,
auditable and safe for public viewing. A separate private academic archive
preserves the source evidence and prior test outcomes.
