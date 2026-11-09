#!/usr/bin/env python3
"""Presentation-only fresh Nav2 goal while RViz records live map + plan.
Preserves existing A/B + reactive trial data. Do not use as independent replicate.
"""
import json, math, sys, subprocess, time
from pathlib import Path
from datetime import datetime, timezone
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult

B=Path(__file__).resolve().parents[1]
OUT=B/"evidence/phase7_rviz_nav_demo_trial01.json"
if OUT.exists():raise SystemExit("Existing evidence, refusing overwrite")
goal=(-0.4,-0.5)
record={"trial":"phase7_rviz_nav_demo_trial01","role":"Visual demo, not independent A/B replicate",
        "goal_map":goal,"start_utc":datetime.now(timezone.utc).isoformat(),"status":"NOT_STARTED"}
def pose():
    p=subprocess.run(["gz","model","-m","waffle","-p"],capture_output=True,text=True,timeout=7)
    if p.returncode:raise RuntimeError(p.stderr[-250:])
    vals=[float(v) for v in p.stdout.strip().split()]
    if len(vals)!=6:raise RuntimeError("Gazebo pose malformed")
    return vals
rclpy.init(args=["--ros-args","-p","use_sim_time:=true"])
nav=BasicNavigator()
try:
    src=pose()
    record["start_gazebo"]=src
    if not (-1.65<src[0]<-1.2 and abs(src[1]+.5)<.12):
        raise RuntimeError(f"Unexpected start for visual demo: {src}")
    for j in range(10):rclpy.spin_once(nav,timeout_sec=.05)
    target=PoseStamped();target.header.frame_id="map"
    target.header.stamp=nav.get_clock().now().to_msg()
    target.pose.position.x=float(goal[0]);target.pose.position.y=float(goal[1])
    target.pose.orientation.w=1.0
    record["status"]="ACTION_SENT"
    print("NAV2_VIDEO_GOAL_SENT",goal,flush=True)
    nav.goToPose(target)
    start=time.monotonic()
    last=0.0
    while not nav.isTaskComplete():
        now=time.monotonic()-start
        if now>55:
            nav.cancelTask();record["canceled_on_timeout"]=True
            raise TimeoutError("Nav2 visual demo exceeds 55 s")
        if now-last>2.0:
            feedback=nav.getFeedback()
            if feedback:
                record["last_feedback_remaining_m"]=round(feedback.distance_remaining,4)
                print("FEEDBACK_REMAINING",record["last_feedback_remaining_m"],flush=True)
            last=now
        time.sleep(.25)
    record["elapsed_wall_seconds"]=round(time.monotonic()-start,3)
    result=nav.getResult()
    record["action_result"]=str(result)
    record["nav2_succeeded"]=bool(result==TaskResult.SUCCEEDED)
    record["final_gazebo"]=pose()
    dx=record["final_gazebo"][0]-goal[0];dy=record["final_gazebo"][1]-goal[1]
    record["unadjusted_world_goal_xy_distance_m"]=round(math.hypot(dx,dy),5)
    record["physical_sanity_pass"]=math.hypot(dx,dy)<.16 and abs(record["final_gazebo"][4])<.12
    record["status"]="PASS" if record["nav2_succeeded"] and record["physical_sanity_pass"] else "FAIL_OR_INCONSISTENT"
except Exception as e:
    record["status"]="BLOCKED";record["error"]=f"{type(e).__name__}: {e}"
finally:
    record["end_utc"]=datetime.now(timezone.utc).isoformat()
    OUT.write_text(json.dumps(record,indent=2)+"\n")
    print("NAV2_GUI_DEMO_VERDICT",record["status"],"REPORT",OUT,flush=True)
    nav.destroy_node();rclpy.shutdown()
sys.exit(0 if record["status"]=="PASS" else 1)
