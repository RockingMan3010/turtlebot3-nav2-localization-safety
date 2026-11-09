#!/usr/bin/env python3
"""Generate evidence PNG charts from recorded ROS2 bags; no fabricated measurements."""
from pathlib import Path
import json
import math
import re
import sqlite3
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BASE = Path(__file__).resolve().parents[1]
OUT = BASE / "evidence"
LOGS = BASE / "logs"
OUT.mkdir(exist_ok=True)

def messages(bag, topic):
    paths = list((LOGS / bag).glob("*.db3"))
    if len(paths) != 1:
        raise RuntimeError(f"Expected exactly one DB3 for {bag}: {paths}")
    con = sqlite3.connect(str(paths[0]))
    row = con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    if not row:
        con.close()
        raise RuntimeError(f"Missing {topic} in {bag}")
    topic_id, typ = row
    msgtype = get_message(typ)
    data = [(t / 1e9, deserialize_message(x, msgtype))
            for t,x in con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))]
    con.close()
    return data

def save(fig, name):
    fig.savefig(OUT/name, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {OUT/name}")

# Chart 1: controlled A/B local odometry XY from recorded trials
fig,ax=plt.subplots(figsize=(9,6))
for cond,trial in [("A",2),("A",3),("B",2),("B",3)]:
    topic="/odom" if cond=="A" else "/odometry/filtered"
    samples=messages(f"phase3_{cond}_trial{trial:02d}_rosbag",topic)
    xx=[m.pose.pose.position.x for _,m in samples]
    yy=[m.pose.pose.position.y for _,m in samples]
    ax.plot(xx,yy,label=f"{cond}{trial}: {'Gazebo odom' if cond=='A' else 'EKF filtered'}", linewidth=1.8)
ax.scatter([-1.4],[-.5],marker="*",s=170,label="map-frame goal (approx.)")
ax.set(xlabel="Local odometry x [m]",ylabel="Local odometry y [m]",
       title="A/B controlled trials: measured local odometry trajectories\n"
             "Local odom frame, not synchronized ground-truth accuracy")
ax.set_aspect("equal",adjustable="datalim")
ax.grid(alpha=.3);ax.legend(fontsize=8);save(fig,"phase3_AB_local_odometry_trajectories.png")

# Chart 2: action wall times for all six completed goals
bars=[]
for cond in ["A","B"]:
    for trial in (1,2,3):
        if trial==1:
            fname="nav_20261105_211508.json" if cond=="A" else "nav_20261105_211749.json"
            action=json.loads((LOGS/fname).read_text())
        else:
            report=json.loads((OUT/f"phase3_{cond}_trial{trial:02d}_report.json").read_text())
            action=report["action"]
        bars.append((f"{cond}{trial}",action["wall_elapsed_s"]))
fig,ax=plt.subplots(figsize=(9,4))
ax.bar([x for x,y in bars],[y for x,y in bars])
ax.set(xlabel="Completed action",ylabel="Wall-clock navigation duration [s]",
       title="Six short-goal Nav2 actions: 3 A + 3 B\n"
             "Small samples: no performance superiority inferred")
ax.set_ylim(0,max(y for _,y in bars)+1)
for i,(_,v) in enumerate(bars):ax.text(i,v+.08,f"{v:.2f}",ha="center",fontsize=9)
ax.grid(alpha=.2,axis="y");save(fig,"phase3_AB_short_goal_action_times.png")

# Chart 3: real local odometry paths for three obstacle experiments
fig,ax=plt.subplots(figsize=(9,6))
conditions=[
    ("reactive_dynamic_trial01_rosbag", "V1 wide: failed", (-1.15,-.5,.30,.60)),
    ("reactive_dynamic_trial02_rosbag", "V2 narrow: false Nav2 success", (-.95,-.5,.18,.24)),
    ("reactive_dynamic_trial03_rosbag", "V3 offset: physical goal sanity PASS", (-1.15,-.32,.16,.16)),
]
for bag,label,(cx,cy,w,h) in conditions:
    smp=messages(bag,"/odom")
    xx=[m.pose.pose.position.x for _,m in smp]
    yy=[m.pose.pose.position.y for _,m in smp]
    line,=ax.plot(xx,yy,label=label,linewidth=1.8)
    ax.scatter(xx[-1],yy[-1],c=[line.get_color()],s=50)
    ax.add_patch(Rectangle((cx-w/2,cy-h/2),w,h,fill=False,
                           edgecolor=line.get_color(),linestyle="--",linewidth=1.5))
ax.scatter([-.4],[-.5],marker="*",s=175,label="goal (map frame)")
ax.set(xlabel="Gazebo odometry x [m]",ylabel="Gazebo odometry y [m]",
       title="Obstacle tests: recorded odometry XY paths and obstacle footprints\n"
             "Map/world approximate for illustration; NOT independent ground truth")
ax.set_xlim(-2.25,0.0);ax.set_ylim(-1.3,.1);ax.set_aspect("equal")
ax.grid(alpha=.25);ax.legend(fontsize=8,loc="lower right");save(fig,"phase4_obstacle_trials_odom_paths.png")

# Chart 4: pre/post planner output in map frame for successful V3
bag="reactive_dynamic_trial03_rosbag"
result=json.loads((LOGS/"reactive_dynamic_trial03.json").read_text())
spawn=float(re.search(r"\[(\d+\.\d+)\].*Spawn status:",result["spawn_log_tail"]).group(1))
plans=messages(bag,"/plan")
pre=[(t,p) for t,p in plans if t<spawn]
post=[(t,p) for t,p in plans if t>spawn]
fig,ax=plt.subplots(figsize=(9,6))
for label,(t,path) in [
    ("Plan before obstacle",pre[0]),
    ("First revised plan (+%.1fs)"%(post[0][0]-spawn),post[0]),
    ("Later revised plan (+%.1fs)"%(post[5][0]-spawn),post[5])]:
    ax.plot([p.pose.position.x for p in path.poses],
            [p.pose.position.y for p in path.poses],label=label,linewidth=2,alpha=.9)
ax.add_patch(Rectangle((-1.15-.08,-.32-.08),.16,.16,
                       facecolor="red",alpha=.4,label="Inserted Gazebo obstacle"))
ax.scatter([-.4],[-.5],marker="*",s=160,label="goal")
ax.set(xlabel="Nav2 map x [m]",ylabel="Nav2 map y [m]",
       title="V3: global plan response to inserted dynamic obstacle")
ax.set_xlim(-2.15,-.2);ax.set_ylim(-1.3,.2);ax.set_aspect("equal")
ax.grid(alpha=.3);ax.legend(fontsize=9);save(fig,"phase4_V3_dynamic_replanning.png")

# Chart 5: V3 commanded velocity and front LiDAR during obstacle encounter
cmd=messages(bag,"/cmd_vel")
scan=messages(bag,"/scan")
t=[v-spawn for v,_ in cmd]
linear=[msg.linear.x for _,msg in cmd]
angular=[msg.angular.z for _,msg in cmd]
range_t=[];front=[]
for stamp,msg in scan:
    ranges=[v for i,v in enumerate(msg.ranges)
            if abs(msg.angle_min+i*msg.angle_increment)<math.radians(18)
            and math.isfinite(v)]
    if ranges:
        range_t.append(stamp-spawn);front.append(min(ranges))
fig,ax=plt.subplots(figsize=(10,5))
ax.plot(t,linear,label="Linear x [m/s]")
ax.plot(t,angular,label="Angular z [rad/s]",alpha=.8)
ax.axvline(0,color="black",linestyle="--",label="Obstacle spawned")
ax.set(xlabel="Time relative to obstacle insertion [s]",ylabel="Velocity command",
       title="V3 reactive command trace: turns and zero-velocity intervals")
ax.grid(alpha=.25);ax.legend(fontsize=9);save(fig,"phase4_V3_stop_turn_velocity.png")
fig,ax=plt.subplots(figsize=(10,4))
ax.plot(range_t,front,label="Minimum LiDAR range within +/-18 deg")
ax.axvline(0,color="black",linestyle="--",label="Obstacle spawned")
ax.set(xlabel="Time relative to obstacle insertion [s]",ylabel="Min frontal LiDAR range [m]",
       title="V3 obstacle approach: actual recorded LaserScan")
ax.grid(alpha=.25);ax.legend(fontsize=9);save(fig,"phase4_V3_lidar_front_range.png")
print("ALL SIX evidence plots generated from actual rosbags.")
