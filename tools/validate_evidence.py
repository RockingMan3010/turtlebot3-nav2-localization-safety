#!/usr/bin/env python3
"""Offline consistency checks for *published* ROS 2 simulation results.

This does not execute ROS, Gazebo or a hardware safety test.
Run: python3 tools/validate_evidence.py
"""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"

def load(name):
    return json.loads((DATA / name).read_text(encoding="utf-8"))

def approximately(got, expected, tol=1e-6):
    assert math.isclose(float(got), float(expected), abs_tol=tol), (got, expected)

bench = load("benchmark_summary.json")
obstacle = load("obstacle_summary.json")
ground_truth = load("ground_truth_summary.json")
with (DATA/"obstacle_trials.csv").open(newline="", encoding="utf-8") as fd:
    trials = list(csv.DictReader(fd))

assert len(trials)==5 and [int(t["trial"]) for t in trials]==[1,2,3,4,5]
assert bench["ab"]["amcl_global_localization_in_both"] is True
for name, successes, mean in (("A",3,4.347),("B",3,4.104)):
    result = bench["ab"][name]
    assert result["success"] == result["trials"] == successes
    approximately(result["mean_s"],mean)

assert trials[0]["nav2_action_result"]=="TaskResult.FAILED"
assert trials[1]["nav2_action_result"]=="TaskResult.SUCCEEDED"
assert trials[1]["physical_pose_sanity"]=="False"
approximately(trials[1]["world_goal_xy_offset_m"],0.85047)
offset = trials[2:]
assert len(offset)==3
assert all(t["setup"]=="offset" for t in offset)
assert all(t["nav2_action_result"]=="TaskResult.SUCCEEDED" and
           t["physical_pose_sanity"]=="True" for t in offset)
assert obstacle["offset_config_successes"]==3
assert bench["dynamic_obstacles"]["offset_success"]==3
approximately(
  sum(float(t["world_goal_xy_offset_m"]) for t in offset)/3,
  obstacle["mean_unadjusted_goal_xy_offset_m"],1e-5)
for label,n,expected in (("A",5,0.01518),("B",6,0.01360)):
    a=bench["ground_truth"][label]
    b=ground_truth["results"][label]
    assert a["aligned_pairs"] == b["time_aligned_pairs"] == n
    approximately(a["xy_rmse_m"],b["map_xy_rmse_m"])
    approximately(a["xy_rmse_m"],expected)
approximately(ground_truth["calibration"]["map_resolution_m"],0.05)

stop = bench["collision_monitor"]
assert "injected" in stop["velocity_source"]
assert stop["hazard_positive_input"]>=20
assert stop["hazard_zero_output"]==21
assert stop["hazard_positive_output"]==0
assert 0<=stop["hazard_settled_travel_m"]<0.001
assert stop["clear_before_travel_m"]>0.15
assert stop["clear_after_travel_m"]>0.2
assert bench["rviz_demo"]["nav2_success"] is True
assert bench["rviz_demo"]["not_independent_ab_trial"] is True

for sdf in (ROOT/"models").rglob("*.sdf"):
    ET.parse(sdf)

assets = [
    ROOT/"assets/demo.mp4",
    ROOT/"assets/gazebo_screenshot.png",
    ROOT/"assets/rviz_screenshot.png",
    *list((ROOT/"assets/figures").glob("*.png"))
]
assert len(assets)>=9
for p in assets:
    assert p.is_file() and p.stat().st_size>1024, p
assert (ROOT/"assets/demo.mp4").stat().st_size<50*1024*1024

print("PASS: Five obstacle trials, A/B metrics, calibrated GT summaries,")
print("      monitored stop/hold/resume, SDF XML and real media assets.")
print("NOTE: This is offline consistency and syntax validation, NOT a ROS runtime test.")
