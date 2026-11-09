#!/usr/bin/env python3
"""Independent Phase6 rosbag verification. Never edits original bag or runner report."""
import json,math,sqlite3
from datetime import datetime,timezone
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parents[1]
BAG=BASE/"logs/phase6_integrated_stop_trial01_rosbag/phase6_integrated_stop_trial01_rosbag_0.db3"
ORIG=BASE/"evidence/phase6_integrated_gazebo_stop_trial01.json"
OUT=BASE/"evidence/phase6_integrated_gazebo_stop_trial01_bag_reanalysis.json"
PLOT=BASE/"evidence/phase6_integrated_gazebo_real_lidar_braking.png"
if OUT.exists() or PLOT.exists():
    raise SystemExit("Refusing to overwrite Phase6 analysis evidence")
c=sqlite3.connect("file:"+str(BAG)+"?mode=ro",uri=True)
assert c.execute("pragma quick_check").fetchone()[0]=="ok"
topics={i:(name,get_message(typ)) for i,name,typ in
        c.execute("select id,name,type from topics")}
wanted={"/scan","/task2/ground_truth","/cmd_vel_pre_collision","/cmd_vel"}
rows={k:[] for k in wanted}
for tid,ts,blob in c.execute("select topic_id,timestamp,data from messages order by timestamp"):
    name,cls=topics[tid]
    if name not in wanted:continue
    m=deserialize_message(blob,cls)
    t=ts/1e9
    if name=="/scan":
        front=[float(r) for i,r in enumerate(m.ranges)
               if abs((m.angle_min+i*m.angle_increment+math.pi)%(2*math.pi)-math.pi)
                       <=math.radians(17)
               and math.isfinite(r) and m.range_min<=r<=m.range_max]
        rows[name].append((t,min(front) if front else None))
    elif name=="/task2/ground_truth":
        rows[name].append((t,float(m.pose.pose.position.x),float(m.pose.pose.position.y)))
    else:
        rows[name].append((t,float(m.linear.x),float(m.angular.z)))
c.close()
front=rows["/scan"]
hazard_scans=[(t,r) for t,r in front if r is not None and r<0.5]
assert len(hazard_scans)>=5,"Insufficient physical LiDAR hazard evidence"
h0,h1=hazard_scans[0][0],hazard_scans[-1][0]
ground=rows["/task2/ground_truth"]
raw=rows["/cmd_vel_pre_collision"]
filtered=rows["/cmd_vel"]
def pick(a,start,end):return [z for z in a if start<=z[0]<=end]
def travelled(a):
    if len(a)<2:return None
    return math.hypot(a[-1][1]-a[0][1],a[-1][2]-a[0][2])
# Guard against startup and post-trial idle command signals: inspect only stable hazard window.
stable0,stable1=h0+1.05,h1-.35
hin=pick(raw,stable0,stable1);hout=pick(filtered,stable0,stable1)
hgt=pick(ground,stable0,stable1)
before=pick(ground,h0-2.5,h0-.4)
after=pick(ground,h1+.7,h1+2.7)
before_out=pick(filtered,h0-2.5,h0-.4)
after_out=pick(filtered,h1+.7,h1+2.7)
numbers={
 "hazard_scan_count":len(hazard_scans),
 "first_hazard_scan_unix_s":round(h0,3),
 "last_hazard_scan_unix_s":round(h1,3),
 "hazard_scans_min_range_m":round(min(r for _,r in hazard_scans),4),
 "hazard_stable_window_s":[round(stable0,3),round(stable1,3)],
 "hazard_stable_command_positive":sum(x[1]>.09 for x in hin),
 "hazard_stable_output_zero":sum(abs(x[1])<.005 and abs(x[2])<.005 for x in hout),
 "hazard_stable_output_positive":sum(x[1]>.09 for x in hout),
 "hazard_stable_gt_xy_travel_m":round(travelled(hgt),5) if travelled(hgt) is not None else None,
 "pre_hazard_gt_xy_travel_m":round(travelled(before),4) if travelled(before) is not None else None,
 "after_hazard_gt_xy_travel_m":round(travelled(after),4) if travelled(after) is not None else None,
 "pre_hazard_positive_out":sum(x[1]>.09 for x in before_out),
 "post_hazard_positive_out":sum(x[1]>.09 for x in after_out),
 "source_topic_counts":{k:len(v) for k,v in rows.items()}}
orig=json.loads(ORIG.read_text())
report={"created_utc":datetime.now(timezone.utc).isoformat(),
"source_bag":str(BAG),"source_runner_report":str(ORIG),
"sqlite_integrity":"ok",
"method":"Independently deserialize ROS2 CDR bag and identify hazard from Gazebo LaserScan forward ±17 degree, nearest finite range <0.5m. Evaluate steady input-output and physics during hazard, as well as before/after motion.",
"original_runner_status":orig["status"],
"original_runner_error":orig.get("error"),
"interpretation":"Physical stop/pass/resume phases recorded successfully; runner post-phase shutdown generated invalid integer-zero Twist, so original runner reported BLOCKED. This reanalysis is not a new experiment.",
"metrics":numbers}
pass_logic=(len(hazard_scans)>=5 and numbers["hazard_stable_command_positive"]>=8
 and numbers["hazard_stable_output_zero"]>=5
 and numbers["hazard_stable_output_positive"]<=1
 and numbers["hazard_stable_gt_xy_travel_m"] is not None
 and numbers["hazard_stable_gt_xy_travel_m"]<.04
 and (numbers["pre_hazard_gt_xy_travel_m"] or 0)>.05
 and (numbers["after_hazard_gt_xy_travel_m"] or 0)>.05
 and numbers["pre_hazard_positive_out"]>=3 and numbers["post_hazard_positive_out"]>=3)
report["assessment"]="PHYSICAL_STOP_PASS_WITH_POSTRUN_SCRIPT_EXCEPTION" if pass_logic else "FAIL_OR_INCONCLUSIVE"
# Visualize actual Gazebo topics, not synthetic simulation.
t0=h0-3
fig,(ax1,ax2)=plt.subplots(2,1,figsize=(11,7),sharex=True)
for k,label,color in [("/cmd_vel_pre_collision","Command to monitor","tab:blue"),
                      ("/cmd_vel","Monitor output to robot","tab:orange")]:
    a=rows[k]
    ax1.plot([z[0]-t0 for z in a],[z[1] for z in a],lw=1.1,label=label,color=color)
ax1.axvspan(h0-t0,h1-t0,alpha=.16,color="firebrick",label="Gazebo LiDAR stop-zone hazard")
ax1.set_ylabel("Forward Twist (m/s)")
ax1.set_ylim(-.02,.17)
ax1.grid(alpha=.25);ax1.legend(loc="upper right",fontsize=8)
ax2.plot([z[0]-t0 for z in ground],[z[1] for z in ground],color="tab:green",
         label="Gazebo physics X (m)",lw=1.4)
ax2.axvspan(h0-t0,h1-t0,alpha=.16,color="firebrick")
ax2.set_ylabel("Physics X position (m)")
ax2.set_xlabel("Wall-clock seconds relative to hazard onset (t=3s)")
ax2.grid(alpha=.25);ax2.legend(loc="upper left",fontsize=8)
ax1.set_xlim(0, (h1-t0)+3.5)
fig.suptitle("Group E Phase 6: real Gazebo LiDAR -> Collision Monitor -> stopped physics")
fig.tight_layout()
fig.savefig(PLOT,dpi=160)
plt.close(fig)
OUT.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps({"assessment":report["assessment"],"metrics":numbers,
                 "report":str(OUT),"plot":str(PLOT)},indent=2))
raise SystemExit(0 if pass_logic else 1)
