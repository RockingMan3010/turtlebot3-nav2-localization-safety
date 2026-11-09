#!/usr/bin/env python3
"""Isolated dynamic obstruction test. Gazebo simulation ONLY, no installed file edits."""
import datetime as dt
import json
import math
import subprocess
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

BASE = Path(__file__).resolve().parents[1]
OBSTACLE = "group_e_reactive_blocker_offset"
GOAL_X, GOAL_Y = -0.4, -0.5

def gazebo_pose(model):
    cp = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=6)
    if cp.returncode:
        return None
    try:
        vals = [float(v) for v in cp.stdout.strip().split()]
        return vals if len(vals) == 6 else None
    except ValueError:
        return None

def stamped_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p

def main():
    out = BASE / "logs/reactive_dynamic_trial03.json"
    data = {"condition": "A", "goal_map": [GOAL_X,GOAL_Y],
            "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "status": "NOT_STARTED"}
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    try:
        start = gazebo_pose("waffle")
        if start is None or abs(start[0] + 2) > 0.10 or abs(start[1] + 0.5) > 0.10:
            raise RuntimeError("Robot not at expected start (-2,-0.5)")
        if gazebo_pose(OBSTACLE) is not None:
            raise RuntimeError("Obstacle pre-exists; fresh world required")
        data["start_gazebo"] = start
        for _ in range(15):
            rclpy.spin_once(nav, timeout_sec=0.05)
        target = stamped_pose(nav, GOAL_X, GOAL_Y, 0)
        source = stamped_pose(nav, start[0], start[1], start[5])
        plan = nav.getPath(source, target, use_start=True)
        if plan is None or len(plan.poses) < 2:
            raise RuntimeError("Cannot get an unobstructed initial path")
        distance = sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
                       for a,b in zip(plan.poses,plan.poses[1:]))
        data["pre_obstacle_plan_metres"] = round(distance,4)
        data["pre_obstacle_waypoints"] = len(plan.poses)
        if distance < 1.0 or distance > 3.0:
            raise RuntimeError(f"Unsafe/invalid test path length {distance}")
        print(f"Pre-obstacle path valid: {distance:.3f}m, {len(plan.poses)} poses",flush=True)
        nav.goToPose(target)
        begun = time.monotonic()
        # The robot starts navigating before the obstacle is inserted.
        time.sleep(1.0)
        data["pose_just_before_insertion"] = gazebo_pose("waffle")
        spawn = ["ros2", "run", "gazebo_ros", "spawn_entity.py",
                 "-entity", OBSTACLE, "-file", str(BASE/"models/reactive_blocker_offset.sdf"),
                 "-x", "-1.15", "-y", "-0.32", "-z", "0.225"]
        result = subprocess.run(spawn, capture_output=True, text=True, timeout=20)
        data["spawn_exit_code"] = result.returncode
        data["spawn_log_tail"] = (result.stdout + result.stderr)[-1400:]
        data["obstacle_pose"] = gazebo_pose(OBSTACLE)
        print(f"Obstacle inserted: exit={result.returncode}, pose={data['obstacle_pose']}",flush=True)
        if result.returncode != 0 or data["obstacle_pose"] is None:
            nav.cancelTask()
            raise RuntimeError("Obstacle insertion failed; navigation canceled")
        while not nav.isTaskComplete():
            if time.monotonic() - begun > 70:
                nav.cancelTask()
                data["timed_out"] = True
                break
            time.sleep(0.25)
        result = nav.getResult()
        data["action_result"] = str(result)
        data["succeeded"] = result == TaskResult.SUCCEEDED
        data["elapsed_wall_seconds"] = round(time.monotonic()-begun,3)
        data["final_gazebo"] = gazebo_pose("waffle")
        # Basic simulated-physical consistency check, NOT independently aligned map-frame RMSE.
        # Pre-trial unblocked control showed the world and map XY coordinates roughly aligned.
        physical_distance = (math.hypot(data["final_gazebo"][0]-GOAL_X,
                                       data["final_gazebo"][1]-GOAL_Y)
                             if data["final_gazebo"] else None)
        data["unadjusted_gazebo_goal_xy_distance_m"] = physical_distance
        data["physical_goal_sanity_pass"] = (physical_distance is not None and physical_distance < 0.16
                                               and abs(data["final_gazebo"][4]) < 0.12)
        data["status"] = ("GOAL_SUCCEEDED_AND_PHYSICAL_SANITY_PASS" if data["succeeded"] and data["physical_goal_sanity_pass"]
                          else "NAV_ACTION_SUCCEEDED_BUT_PHYSICAL_POSE_INCONSISTENT" if data["succeeded"]
                          else "GOAL_NOT_SUCCEEDED")
        print(f"Navigation result: {data['status']}; wall={data['elapsed_wall_seconds']}s",flush=True)
    except Exception as exc:
        data["status"] = "BLOCKED"
        data["error"] = f"{type(exc).__name__}: {exc}"
        print("BLOCKED:",data["error"],flush=True)
    finally:
        out.write_text(json.dumps(data,indent=2)+"\n")
        print(f"SAVED: {out}",flush=True)
        nav.destroy_node()
        rclpy.shutdown()
    return 0 if data["status"] == "GOAL_SUCCEEDED_AND_PHYSICAL_SANITY_PASS" else 1

if __name__ == "__main__":
    raise SystemExit(main())
