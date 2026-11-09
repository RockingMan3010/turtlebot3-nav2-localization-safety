#!/usr/bin/env python3
"""Safe, reproducible fresh-launch Nav2 trial for AMR Group E Task 2.
One condition/trial per invocation. No installed ROS files touched.
"""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

BASE = Path(__file__).resolve().parents[1]
LOGS = BASE / "logs"
EVIDENCE = BASE / "evidence"
GOAL = (-1.4, -0.5)

def run(args, timeout=10):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)

def safe_shutdown(proc, label, timeout=20):
    if proc is None or proc.poll() is not None:
        return
    print(f"Stopping {label} with SIGINT", flush=True)
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"WARNING: {label} did not respond to SIGINT; SIGTERM to owned process only", flush=True)
        proc.terminate()
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            print(f"BLOCKER: {label} still alive, manual inspection required", flush=True)

def clean_environment():
    for _ in range(24):
        ps = run(["pgrep", "-x", "gzserver"], 4)
        ekf = run(["pgrep", "-x", "ekf_node"], 4)
        graph = run(["ros2", "node", "list"], 8)
        if ps.returncode and ekf.returncode and not any(
            x.strip() in graph.stdout.splitlines()
            for x in ["/amcl", "/bt_navigator", "/controller_server", "/ekf_assisted", "/ekf_shadow"]
        ):
            return True
        time.sleep(1)
    return False

def wait_gazebo(launch, deadline=50):
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        if launch.poll() is not None:
            raise RuntimeError("Launch process exited before Gazebo spawned")
        pose = run(["gz", "model", "-m", "waffle", "-p"], 5)
        if pose.returncode == 0:
            try:
                numbers = [float(x) for x in pose.stdout.strip().split()]
                if len(numbers) == 6 and abs(numbers[0] + 2.0) < 0.1 and abs(numbers[1] + 0.5) < 0.1:
                    return numbers
            except ValueError:
                pass
        time.sleep(1)
    raise RuntimeError("Gazebo robot at expected spawn pose not observed within deadline")

def initialize_amcl(log):
    for attempt in range(1, 5):
        x = run(["python3", str(BASE / "scripts/initialize_amcl_from_gazebo.py"), "--apply"], 19)
        log.write_text(log.read_text() + f"INIT attempt {attempt}:\n{x.stdout}\n{x.stderr}\n")
        if x.returncode == 0 and "request delivered" in x.stdout:
            return
        time.sleep(2)
    raise RuntimeError("AMCL initialization not acknowledged")

def wait_nav2(launch, deadline=65):
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        if launch.poll() is not None:
            raise RuntimeError("Launch process exited before Nav2 activation")
        nav = run(["ros2", "lifecycle", "get", "/bt_navigator"], 6)
        planner = run(["ros2", "lifecycle", "get", "/planner_server"], 6)
        if "active [3]" in nav.stdout and "active [3]" in planner.stdout:
            return
        time.sleep(2)
    raise RuntimeError("Nav2 did not become active")

def verify_wiring(condition):
    odom = run(["ros2", "param", "get", "/controller_server", "odom_topic"], 6).stdout
    bt = run(["ros2", "param", "get", "/bt_navigator", "odom_topic"], 6).stdout
    tf = run(["ros2", "topic", "info", "-v", "/tf"], 7).stdout.split("Subscription count:")[0]
    expected = "/odom" if condition == "A" else "/odometry/filtered"
    if expected not in odom or expected not in bt:
        raise RuntimeError(f"Incorrect Nav2 odom topic: controller={odom} bt={bt}")
    if condition == "A":
        if "Node name: turtlebot3_diff_drive" not in tf or "Node name: ekf_assisted" in tf:
            raise RuntimeError("Condition A incorrect TF broadcasters")
    else:
        ekf = run(["ros2", "param", "get", "/ekf_assisted", "publish_tf"], 6).stdout
        if "True" not in ekf or "Node name: ekf_assisted" not in tf or "Node name: turtlebot3_diff_drive" in tf:
            raise RuntimeError(f"Condition B incorrect TF broadcasters or publish_tf: {ekf} / {tf}")
        filt = run(["ros2", "topic", "echo", "/odometry/filtered", "--once", "--field", "header"], 8)
        if filt.returncode != 0 or "frame_id: odom" not in filt.stdout:
            raise RuntimeError("Condition B filtered odometry message not verified")
    return {"controller_odom": odom.strip(), "bt_odom": bt.strip(), "tf_publishers": tf.strip()}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--condition", choices=["A", "B"], required=True)
    ap.add_argument("--trial", type=int, required=True)
    a = ap.parse_args()
    tag = f"phase3_{a.condition}_trial{a.trial:02d}"
    EVIDENCE.mkdir(exist_ok=True)
    LOGS.mkdir(exist_ok=True)
    report = {"trial": tag, "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "goal_map": list(GOAL), "start_intended": [-2.0, -0.5, 0.0],
              "status": "NOT_STARTED"}
    launch = bag = None
    result_path = EVIDENCE / (tag + "_report.json")
    launch_log = LOGS / (tag + "_launch.log")
    trial_log = LOGS / (tag + "_action.log")
    init_log = LOGS / (tag + "_init.log")
    init_log.write_text("")
    try:
        if not clean_environment():
            raise RuntimeError("Preexisting Gazebo/Nav2/EKF detected. Refusing trial.")
        startup = BASE / "scripts" / ("start_phase3_amcl_A.sh" if a.condition == "A" else "start_phase3_ekf_assisted.sh")
        fp = launch_log.open("w")
        try:
            launch = subprocess.Popen(["bash", str(startup)], stdout=fp, stderr=subprocess.STDOUT)
        finally:
            fp.close()
        print(f"{tag}: launching fresh simulator", flush=True)
        report["spawn"] = wait_gazebo(launch)
        print(f"{tag}: Gazebo spawn verified", flush=True)
        initialize_amcl(init_log)
        wait_nav2(launch)
        report["runtime"] = verify_wiring(a.condition)
        print(f"{tag}: AMCL/Nav2/TF wiring verified", flush=True)
        cmd = ["python3", str(BASE / "scripts/nav_goal_trial.py"),
               "--goal-x", str(GOAL[0]), "--goal-y", str(GOAL[1])]
        # Wait for map -> odom and Nav2 action server to settle, then retry
        # read-only planning checks if DDS returns a transient timeout.
        # No goal movement occurs until a verified preflight succeeds.
        initialize_amcl(init_log)
        time.sleep(3)
        preflight = None
        for attempt in range(1, 3):
            try:
                preflight = run(cmd, 38)
                output = preflight.stdout + preflight.stderr
            except subprocess.TimeoutExpired as exc:
                output = "PLANNING TIMEOUT: " + str(exc)
                preflight = None
            (LOGS / f"{tag}_preflight_attempt{attempt}.log").write_text(output)
            if preflight and preflight.returncode == 0 and "PLAN_VALID_NO_MOVEMENT" in preflight.stdout:
                report["preflight_attempts"] = attempt
                break
            if "NO_VALID_GLOBAL_PATH" in output or "PLAN_LENGTH_OUTSIDE_SAFE_TEST_RANGE" in output:
                raise RuntimeError("Safety preflight rejected path: " + output[-700:])
            print(f"{tag}: preflight attempt {attempt} inconclusive; reinitializing AMCL", flush=True)
            if attempt < 2:
                initialize_amcl(init_log)
                time.sleep(4)
        else:
            raise RuntimeError("Two navigation preflight attempts failed; see saved logs")
        report["preflight"] = "PASS"
        topics = ["/clock", "/odom", "/imu", "/scan", "/amcl_pose",
                  "/tf", "/tf_static", "/plan", "/cmd_vel"]
        if a.condition == "B":
            topics.append("/odometry/filtered")
        bag_target = LOGS / f"{tag}_rosbag"
        bfp = (LOGS / f"{tag}_rosbag.log").open("w")
        try:
            bag = subprocess.Popen(["ros2", "bag", "record", "-s", "sqlite3",
                                    "-o", str(bag_target), *topics],
                                   stdout=bfp, stderr=subprocess.STDOUT)
        finally:
            bfp.close()
        time.sleep(3)
        if bag.poll() is not None:
            report["rosbag"] = "FAILED_TO_START"
            bag = None
            raise RuntimeError("Bag recorder exited unexpectedly")
        print(f"{tag}: preflight PASS, recorder running, sending same goal", flush=True)
        action = run(cmd + ["--execute", "--timeout", "75"], 95)
        trial_log.write_text(action.stdout + action.stderr)
        if action.returncode != 0 or "SAVED:" not in action.stdout:
            raise RuntimeError("Goal action script failed: " + action.stdout[-800:] + action.stderr[-300:])
        match = re.search(r"SAVED:\s*(\S+\.json)", action.stdout)
        if not match:
            raise RuntimeError("Cannot locate Nav2 JSON evidence")
        data = json.loads(Path(match.group(1)).read_text())
        report["action"] = data
        report["status"] = "GOAL_SUCCEEDED" if data.get("succeeded") else "GOAL_NOT_SUCCEEDED"
        print(f"{tag}: {report['status']}; elapsed={data.get('wall_elapsed_s')}s", flush=True)
    except Exception as exc:
        report["status"] = "BLOCKED"
        report["error"] = f"{type(exc).__name__}: {exc}"
        print(f"{tag}: BLOCKED {report['error']}", flush=True)
    finally:
        safe_shutdown(bag, "rosbag")
        safe_shutdown(launch, "trial launch")
        report["ended_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        report["rosbag_path"] = str(LOGS / f"{tag}_rosbag")
        result_path.write_text(json.dumps(report, indent=2) + "\n")
        print(f"{tag}: evidence saved to {result_path}", flush=True)
    return 0 if report["status"] == "GOAL_SUCCEEDED" else 1

if __name__ == "__main__":
    sys.exit(main())
