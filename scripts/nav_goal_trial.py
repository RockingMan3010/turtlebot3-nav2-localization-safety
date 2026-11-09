#!/usr/bin/env python3
"""Nav2 map-frame plan preflight and optional small simulation navigation trial."""
import argparse
import datetime as dt
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

BASE = Path(__file__).resolve().parents[1]

def make_pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p

def main():
    a = argparse.ArgumentParser()
    a.add_argument("--goal-x", type=float, default=-1.0)
    a.add_argument("--goal-y", type=float, default=-0.5)
    a.add_argument("--execute", action="store_true")
    a.add_argument("--timeout", type=float, default=75.0)
    args = a.parse_args()

    raw = subprocess.run(["gz", "model", "-m", "waffle", "-p"],
                         text=True, capture_output=True, check=True, timeout=5)
    x, y, _, _, _, yaw = map(float, raw.stdout.split())
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    result = {"recorded_at": dt.datetime.now(dt.timezone.utc).isoformat(),
              "start_gazebo": [x, y, yaw],
              "goal_map": [args.goal_x, args.goal_y, 0.0],
              "plan_only": not args.execute}
    try:
        # Let sim clock and localization settle before requesting a plan.
        for _ in range(15):
            rclpy.spin_once(nav, timeout_sec=0.05)
        start = make_pose(nav, x, y, yaw)
        target = make_pose(nav, args.goal_x, args.goal_y, 0)
        path = nav.getPath(start, target, use_start=True)
        if path is None or len(path.poses) < 2:
            result["result"] = "NO_VALID_GLOBAL_PATH"
            print(result)
            return
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                b.pose.position.y - a.pose.position.y)
                     for a, b in zip(path.poses, path.poses[1:]))
        result["planned_path_length_m"] = round(length, 4)
        result["planned_waypoints"] = len(path.poses)
        print(f"GLOBAL PLAN: {len(path.poses)} waypoints, {length:.3f} m", flush=True)
        if length > 4.0 or length < 0.08:
            result["result"] = "PLAN_LENGTH_OUTSIDE_SAFE_TEST_RANGE"
            print(result)
            return
        if not args.execute:
            result["result"] = "PLAN_VALID_NO_MOVEMENT"
            print(result)
            return
        print("Sending monitored simulation goal ...", flush=True)
        nav.goToPose(target)
        begun = time.monotonic()
        while not nav.isTaskComplete():
            if time.monotonic() - begun > args.timeout:
                print("Timeout: canceling goal.", flush=True)
                nav.cancelTask()
                result["timed_out"] = True
                break
            time.sleep(0.25)
        outcome = nav.getResult()
        result["result"] = str(outcome)
        result["succeeded"] = outcome == TaskResult.SUCCEEDED
        result["wall_elapsed_s"] = round(time.monotonic() - begun, 3)
        raw2 = subprocess.run(["gz", "model", "-m", "waffle", "-p"],
                              text=True, capture_output=True, timeout=5, check=True)
        result["final_gazebo"] = list(map(float, raw2.stdout.split()))
        print("FINAL:", result, flush=True)
    finally:
        out = BASE / "logs" / ("nav_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
        out.write_text(json.dumps(result, indent=2))
        print("SAVED:", out, flush=True)
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
