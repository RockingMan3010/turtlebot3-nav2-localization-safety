#!/usr/bin/env python3
"""Phase 7 (visual recording): real Gazebo physics + real 5 Hz LaserScan + ACTIVE Nav2 Collision Monitor.
Commands injected at post-smoother monitor input while Nav2 is idle.
This verifies full sensor -> monitor -> diff-drive -> physics; not Nav2 goal autonomy.
"""
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

BASE = Path(__file__).resolve().parents[1]
OUT = BASE / "evidence/phase7_gazebo_gui_demo_trial01.json"
MODEL = "group_e_phase7_gui_stop_obstacle"
if OUT.exists():
    raise SystemExit("Protected existing Phase6 report; refusing overwrite")
rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
n = rclpy.create_node("group_e_phase7_gui_capture_evaluator")
pub = n.create_publisher(Twist, "/cmd_vel_pre_collision", 10)
phase = ["warmup"]
t0 = time.monotonic()
gt, inp, out, scans = [], [], [], []
def ts(): return round(time.monotonic() - t0, 4)
def odom_cb(m):
    gt.append({"t":ts(), "phase":phase[0], "x":round(m.pose.pose.position.x,5),
               "y":round(m.pose.pose.position.y,5),
               "vx":round(m.twist.twist.linear.x,5)})
def scan_cb(m):
    vals = []
    for i,r in enumerate(m.ranges):
        a = (m.angle_min + i*m.angle_increment + math.pi)%(2*math.pi)-math.pi
        if abs(a) <= math.radians(17) and math.isfinite(r) and m.range_min <= r <= m.range_max:
            vals.append(r)
    scans.append({"t":ts(), "phase":phase[0],
                  "front_min_m":round(min(vals),4) if vals else None})
def in_cb(m): inp.append({"t":ts(),"phase":phase[0],"v":round(m.linear.x,4)})
def out_cb(m): out.append({"t":ts(),"phase":phase[0],"v":round(m.linear.x,4),
                           "w":round(m.angular.z,4)})
subs = [
 n.create_subscription(Odometry, "/task2/ground_truth", odom_cb, qos_profile_sensor_data),
 n.create_subscription(LaserScan, "/scan", scan_cb, qos_profile_sensor_data),
 n.create_subscription(Twist, "/cmd_vel_pre_collision", in_cb, 10),
 n.create_subscription(Twist, "/cmd_vel", out_cb, 10)]
report = {"trial":"phase7_gazebo_gui_demo_trial01",
 "created_utc":datetime.now(timezone.utc).isoformat(),
 "configuration":"Gazebo Classic TB3 Waffle, Nav2 ACTIVE but idle, velocity injected at post-smoother input",
 "sensor":"real Gazebo LaserScan /scan, NOT synthetic",
 "actuator":"Gazebo differential drive /cmd_vel with physics ground truth /task2/ground_truth",
 "command_m_s":0.12,"obstacle_center_world":[-1.35,-0.50],
 "stop_polygon_x_m":[0.10,0.50], "status":"RUNNING"}
obstacle_spawned=False
def spin(seconds, velocity):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        v=Twist(); v.linear.x=velocity; pub.publish(v)
        rclpy.spin_once(n,timeout_sec=0.075)
        time.sleep(.015)
def rpc(cli, req, label, timeout=14):
    if not cli.wait_for_service(timeout_sec=7):
        raise RuntimeError(f"{label} service unavailable")
    f=cli.call_async(req)
    end=time.monotonic()+timeout
    while not f.done() and time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
    if not f.done(): raise TimeoutError(f"{label} timed out")
    if f.exception():raise RuntimeError(f"{label}: {f.exception()}")
    if not f.result().success:
        raise RuntimeError(f"{label}: {f.result().status_message}")
    return f.result().status_message
def subset(rows,p,settle):
    ph=[q for q in rows if q["phase"]==p]
    if not ph:return []
    start=ph[0]["t"]
    return [q for q in ph if q["t"]-start>=settle]
def analyze(p,settle):
    g=subset(gt,p,settle); v=subset(out,p,settle); i=subset(inp,p,settle); s=subset(scans,p,settle)
    dist=math.hypot(g[-1]["x"]-g[0]["x"],g[-1]["y"]-g[0]["y"]) if len(g)>=2 else None
    q={"gt_samples":len(g),"input_count":len(i),"output_count":len(v),
       "output_positive":sum(x["v"]>.09 for x in v),
       "output_zero":sum(abs(x["v"])<.005 and abs(x["w"])<.005 for x in v),
       "input_positive":sum(x["v"]>.09 for x in i),
       "gt_xy_displacement_m":round(dist,4) if dist is not None else None,
       "gt_first_xy":[g[0]["x"],g[0]["y"]] if g else None,
       "gt_last_xy":[g[-1]["x"],g[-1]["y"]] if g else None,
       "front_scan_count":len(s),
       "front_min_m":min([x["front_min_m"] for x in s if x["front_min_m"] is not None],default=None)}
    return q
try:
    start=time.monotonic()
    while time.monotonic()-start<4:
        spin(.2,0.0)
        if len(gt)>=10 and len(scans)>=4: break
    if len(gt)<10 or len(scans)<4:
        raise RuntimeError(f"physics/scans unavailable: GT={len(gt)} scan={len(scans)}")
    if not (-2.10<gt[-1]["x"]<-1.90):
        raise RuntimeError(f"unexpected robot start pose {gt[-1]}")
    phase[0]="clear_before"
    spin(2.0,0.12)
    print("CLEAR_BEFORE",analyze("clear_before",.35),flush=True)
    if analyze("clear_before",.35)["gt_xy_displacement_m"] is None:
        raise RuntimeError("No physical movement evidence in clear phase")
    req=SpawnEntity.Request()
    req.name=MODEL
    req.xml=(BASE/"models/reactive_blocker_offset.sdf").read_text()
    req.robot_namespace=""
    req.reference_frame="world"
    req.initial_pose.position.x=-1.35
    req.initial_pose.position.y=-.50
    req.initial_pose.position.z=.225
    phase[0]="spawn_wait"
    spawn_cli=n.create_client(SpawnEntity,"/spawn_entity")
    report["spawn_result"]=rpc(spawn_cli,req,"spawn")
    obstacle_spawned=True
    print("OBSTACLE_SPAWNED",report["spawn_result"],flush=True)
    phase[0]="hazard"
    spin(3.8,0.12)
    print("HAZARD",analyze("hazard",.95),flush=True)
    req2=DeleteEntity.Request(); req2.name=MODEL
    phase[0]="delete_wait"
    del_cli=n.create_client(DeleteEntity,"/delete_entity")
    report["delete_result"]=rpc(del_cli,req2,"delete")
    obstacle_spawned=False
    print("OBSTACLE_DELETED",report["delete_result"],flush=True)
    phase[0]="clear_after"
    spin(2.6,0.12)
    print("CLEAR_AFTER",analyze("clear_after",.40),flush=True)
    phase[0]="cooldown"
    spin(.7,0.0)
    p={k:analyze(k,s) for k,s in [
      ("clear_before",.35),("hazard",.95),("clear_after",.40)]}
    report["phases"]=p
    a,b,c=p["clear_before"],p["hazard"],p["clear_after"]
    report["status"]="PASS" if (
       a["output_positive"]>=5 and (a["gt_xy_displacement_m"] or 0)>.07
       and b["output_zero"]>=6 and b["input_positive"]>=8
       and b["output_positive"]<=2 and (b["gt_xy_displacement_m"] or 999)<.06
       and c["output_positive"]>=5 and (c["gt_xy_displacement_m"] or 0)>.07
       and b["front_min_m"] is not None and b["front_min_m"]<.5) else "FAIL"
except Exception as e:
    report["status"]="BLOCKED"
    report["error"]=f"{type(e).__name__}: {e}"
    print("ERROR",report["error"],flush=True)
finally:
    phase[0]="shutdown"
    try:spin(.7,0.0)
    except Exception:pass
    if obstacle_spawned:
        try:
            req=DeleteEntity.Request();req.name=MODEL
            report["cleanup_delete"]=rpc(n.create_client(DeleteEntity,"/delete_entity"),req,"cleanup")
        except Exception as e:report["cleanup_delete_error"]=str(e)
    report["last_physics"] = gt[-1] if gt else None
    report["data_counts"]={"physics":len(gt),"scan":len(scans),"input":len(inp),"output":len(out)}
    report["sample_trace"]={"gt":gt[::5],"scan":scans,"input":inp[::3],"output":out[::3]}
    report["ended_utc"]=datetime.now(timezone.utc).isoformat()
    OUT.write_text(json.dumps(report,indent=2)+"\n")
    print("PHASE6_VERDICT",report["status"],"EVIDENCE",OUT,flush=True)
    n.destroy_node()
    rclpy.shutdown()
sys.exit(0 if report["status"]=="PASS" else 1)
