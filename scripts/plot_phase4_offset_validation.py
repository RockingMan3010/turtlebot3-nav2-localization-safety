#!/usr/bin/env python3
"""Offline plots from already recorded successful offset-obstacle trials 3-5."""
from pathlib import Path
import sqlite3
import json
import math
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path(__file__).resolve().parents[1]
LOG=ROOT/"logs"
EVID=ROOT/"evidence"

def load(n, topic):
    folder=LOG/f"reactive_dynamic_trial{n:02}_rosbag"
    db=list(folder.glob("*.db3"))
    if len(db)!=1: raise ValueError(f"Bad bag {folder}")
    con=sqlite3.connect(str(db[0]))
    i,typ=con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    clas=get_message(typ)
    values=[(ns/1e9,deserialize_message(blob,clas)) for ns,blob in
            con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(i,))]
    con.close()
    return values

fig,ax=plt.subplots(figsize=(9,6))
for n in (3,4,5):
    poses=load(n,"/odom")
    xx=[m.pose.pose.position.x for _,m in poses]
    yy=[m.pose.pose.position.y for _,m in poses]
    ln,=ax.plot(xx,yy,lw=1.7,label=f"Offset Trial {n}")
    ax.scatter(xx[-1],yy[-1],s=55,color=ln.get_color())
ax.add_patch(Rectangle((-1.23,-.40),.16,.16,facecolor="#d62828",alpha=.35,label="Inserted obstacle footprint"))
ax.scatter([-.4],[-.5],marker="*",s=200,color="black",label="Requested map goal")
ax.set(title="Offset-obstacle trials 3-5: measured odometry paths",
       xlabel="Local odometry X [m]",ylabel="Local odometry Y [m]")
ax.set_aspect("equal",adjustable="box");ax.set_xlim(-2.18,-.18);ax.set_ylim(-1.5,.22)
ax.grid(alpha=.3);ax.legend(loc="lower right",fontsize=9)
fig.text(.1,.01,"Visualization only: map, odom and Gazebo world alignment not independently validated.",fontsize=8)
dest=EVID/"phase4_offset_trials_3to5_odom_paths.png";fig.savefig(dest,dpi=170,bbox_inches="tight");plt.close(fig);print("SAVED",dest)

rows=[json.loads((LOG/f"reactive_dynamic_trial{n:02}.json").read_text()) for n in (3,4,5)]
fig,ax=plt.subplots(figsize=(8,4))
labels=["T3","T4","T5"]
d=[p["unadjusted_gazebo_goal_xy_distance_m"]*100 for p in rows]
bars=ax.bar(labels,d)
ax.axhline(16,ls="--",label="Conservative physical-pose sanity bound (16 cm)",color="gray")
for i,bar in enumerate(bars):
    ax.text(bar.get_x()+bar.get_width()/2,d[i]+.5,f"{d[i]:.2f} cm",ha="center",fontsize=10)
ax.set(title="Offset-obstacle runs: final unadjusted Gazebo XY separation",
       xlabel="Trial",ylabel="Goal XY separation [cm]")
ax.set_ylim(0,18);ax.grid(alpha=.2,axis="y");ax.legend(fontsize=9)
fig.text(.11,.01,"Not synchronized ground-truth localization error; world/map XY correspondence assumed.",fontsize=8)
dest=EVID/"phase4_offset_trials_3to5_final_gazebo_xy.png";fig.savefig(dest,dpi=170,bbox_inches="tight");plt.close(fig);print("SAVED",dest)
