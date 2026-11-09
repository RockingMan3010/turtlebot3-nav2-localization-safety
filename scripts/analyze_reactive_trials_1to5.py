#!/usr/bin/env python3
"""Offline QA summary for already recorded Task 2 reactive trials 1-5.
No Gazebo/ROS nodes are started. Outputs are derived from saved JSON & sqlite3 ROS bags.
"""
from pathlib import Path
import csv
import json
import math
import re
import sqlite3
import statistics
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

ROOT=Path(__file__).resolve().parents[1]
LOG=ROOT/"logs"
OUT=ROOT/"evidence"

def read_bag(idx):
    folder=LOG/f"reactive_dynamic_trial{idx:02}_rosbag"
    meta=yaml.safe_load((folder/"metadata.yaml").read_text())["rosbag2_bagfile_information"]
    dbs=list(folder.glob("*.db3"))
    if len(dbs)!=1: raise RuntimeError(f"Bag DB3 count mismatch trial {idx}")
    con=sqlite3.connect(str(dbs[0]))
    topics={name:(i,get_message(msgtype)) for i,name,msgtype in con.execute("SELECT id,name,type FROM topics")}
    return con,topics,meta

def decode(con,topics,topic):
    i,typ=topics[topic]
    return [(stamp/1e9,deserialize_message(blob,typ)) for stamp,blob in
            con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(i,))]

def trial_metrics(idx):
    path=LOG/f"reactive_dynamic_trial{idx:02}.json"
    result=json.loads(path.read_text())
    con, topics, meta=read_bag(idx)
    try:
        spawn_log=result.get("spawn_log_tail","")
        match=re.search(r"\[(\d+\.\d+)\].*Spawn status:",spawn_log)
        if not match: raise ValueError(f"Cannot read exact insertion timestamp from trial {idx}")
        inserted=float(match.group(1))
        scans=decode(con,topics,"/scan")
        plans=decode(con,topics,"/plan")
        commands=decode(con,topics,"/cmd_vel")
        yaw_commands=decode(con,topics,"/cmd_vel_nav")
        costmaps=decode(con,topics,"/global_costmap/costmap")

        pre_scan=[];post_scan=[]
        for t,msg in scans:
            front=[v for i,v in enumerate(msg.ranges)
                   if abs(msg.angle_min+i*msg.angle_increment)<=math.radians(18) and
                   math.isfinite(v) and msg.range_min<=v<=msg.range_max]
            if front:
                (pre_scan if t<inserted else post_scan).append(min(front))
        post_cmd=[m for t,m in commands if t>=inserted]
        pre_plans=[(t,m) for t,m in plans if t<inserted]
        post_plans=[(t,m) for t,m in plans if t>=inserted]
        obx,oby,_,_,_,_=result["obstacle_pose"]
        cost_values=[]
        for stamp,grid in costmaps:
            if stamp<inserted:continue
            g=grid.info
            px=int((obx-g.origin.position.x)/g.resolution)
            py=int((oby-g.origin.position.y)/g.resolution)
            if 0<=px<g.width and 0<=py<g.height:
                cost_values.append(int(grid.data[py*g.width+px]))
        goal=result["goal_map"]
        final=result.get("final_gazebo")
        worldxy=math.hypot(final[0]-goal[0],final[1]-goal[1]) if final else None
        sanity=worldxy is not None and worldxy<.16 and abs(final[4])<.12
        topic_counts={v["topic_metadata"]["name"]:v["message_count"] for v in meta["topics_with_message_count"]}
        mandatory={"/scan","/plan","/cmd_vel","/odom","/tf","/clock","/local_costmap/costmap","/global_costmap/costmap"}
        bag_ok=meta["message_count"]>0 and all(topic_counts.get(x,0)>0 for x in mandatory)
        return {
            "trial":idx,
            "setup":"wide_center" if idx==1 else "narrow_center" if idx==2 else "offset",
            "nav2_action_result":result.get("action_result"),
            "world_goal_xy_offset_m":round(worldxy,5) if worldxy is not None else None,
            "physical_pose_sanity":sanity,
            "final_pitch_rad":final[4] if final else None,
            "elapsed_wall_s":result.get("elapsed_wall_seconds"),
            "preplanned_m":result.get("pre_obstacle_plan_metres"),
            "obstacle_world_x":obx,"obstacle_world_y":oby,
            "rosbag_messages":meta["message_count"],
            "rosbag_duration_s":round(meta["duration"]["nanoseconds"]/1e9,3),
            "bag_topic_qa":bag_ok,
            "pre_insertion_plan_messages":len(pre_plans),
            "post_insertion_plan_messages":len(post_plans),
            "post_insertion_cmd_vel_messages":len(post_cmd),
            "post_insertion_stop_command_samples":sum(abs(m.linear.x)<.01 and abs(m.angular.z)<.01 for m in post_cmd),
            "post_insertion_turn_command_samples":sum(abs(m.angular.z)>=.1 for m in post_cmd),
            "post_insertion_nav_turn_commands":sum(abs(m.angular.z)>=.1 for t,m in yaw_commands if t>=inserted),
            "min_frontal_lidar_pre_m":round(min(pre_scan),3) if pre_scan else None,
            "min_frontal_lidar_post_m":round(min(post_scan),3) if post_scan else None,
            "post_obstacle_cost_cell_max":max(cost_values) if cost_values else None,
        }
    finally:
        con.close()

def main():
    data=[trial_metrics(i) for i in (1,2,3,4,5)]
    assert all(x["bag_topic_qa"] for x in data), "Cannot label bag QA passed"
    # Not a test of ground-truth pose accuracy; world-to-map alignment remains an assumption.
    assert all(x["nav2_action_result"]=="TaskResult.SUCCEEDED" and x["physical_pose_sanity"] for x in data[2:]), "Offset configuration did not pass all trials"
    out=OUT/"phase4_reactive_trial1to5_offline_metrics.json"
    out.write_text(json.dumps({"note":"Computed from saved bags and JSON. World->map XY alignment NOT independently validated; physical pose sanity is approximate.","trials":data},indent=2)+"\n")
    table=OUT/"phase4_reactive_trial1to5_offline_metrics.csv"
    with table.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=data[0].keys());w.writeheader();w.writerows(data)
    passed=[x for x in data if x["setup"]=="offset"]
    summary={
        "offset_config_successes":len(passed),
        "offset_config_attempts":len(passed),
        "mean_offset_trial_wall_time_sec":round(statistics.mean(x["elapsed_wall_s"] for x in passed),3),
        "mean_unadjusted_goal_xy_offset_m":round(statistics.mean(x["world_goal_xy_offset_m"] for x in passed),5),
        "max_unadjusted_goal_xy_offset_m":round(max(x["world_goal_xy_offset_m"] for x in passed),5),
        "max_pitch_rad":round(max(abs(x["final_pitch_rad"]) for x in passed),5),
        "offset_config_post_obstacle_plans":[x["post_insertion_plan_messages"] for x in passed],
        "offset_config_bag_qa_pass":all(x["bag_topic_qa"] for x in passed),
        "caution":"Three trials only; no statistical guarantee, independent truth alignment, collision monitor certification, or dynamic-obstacle performance comparison."
    }
    (OUT/"phase4_reactive_verified_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    for row in data:
        print("Trial",row["trial"],row["setup"],row["nav2_action_result"],
              "pose_sanity",row["physical_pose_sanity"],
              "worldXY",row["world_goal_xy_offset_m"],
              "post_plans",row["post_insertion_plan_messages"],
              "stop_samples",row["post_insertion_stop_command_samples"],
              "turn_samples",row["post_insertion_turn_command_samples"],
              "bag_ok",row["bag_topic_qa"])
    print("SUMMARY:",json.dumps(summary))
    print("SAVED:",out,table,OUT/"phase4_reactive_verified_summary.json")

if __name__=="__main__":
    main()
