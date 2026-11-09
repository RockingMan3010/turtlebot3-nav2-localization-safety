#!/usr/bin/env python3
"""Independent Gazebo-P3D reference localization QA for new Phase 5 A/B bags.

P3D is produced directly by physics, not AMCL. Validate map/world registration
against geometry of nine *static* Gazebo cylinder landmarks, not against robot pose.
Compare AMCL observations at their header.stamp against interpolated GT stamps.
"""
from pathlib import Path
import json,math,sqlite3
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
EVID=ROOT/"evidence";LOG=ROOT/"logs"
RES=0.05;ORIGIN=(-10.0,-10.0)
centers=np.asarray([(x,y) for x in (-1.1,0.0,1.1) for y in (-1.1,0.0,1.1)],float)
mp=np.array(Image.open("/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.pgm"),dtype=np.uint8)
# Distance in metres to nearest occupied cell in map coordinate system.
dt=distance_transform_edt(mp!=0)*RES
H,W=mp.shape
angles=np.linspace(0,2*math.pi,48,endpoint=False)
template=np.stack([centers[:,None,0]+.15*np.cos(angles),centers[:,None,1]+.15*np.sin(angles)],axis=-1).reshape(-1,2)
def boundary_error(tx,ty,ang):
 c,s=math.cos(ang),math.sin(ang)
 pts=np.stack([c*template[:,0]-s*template[:,1]+tx,s*template[:,0]+c*template[:,1]+ty],axis=-1)
 u=np.rint((pts[:,0]-ORIGIN[0])/RES).astype(int)
 v=H-1-np.rint((pts[:,1]-ORIGIN[1])/RES).astype(int)
 if np.any(u<0)|np.any(v<0)|np.any(u>=W)|np.any(v>=H):return 999.0
 vals=dt[v,u]
 return float(np.median(vals)),float(np.mean(vals))
best=None
for theta_deg in np.arange(-5,5.1,0.5):
 for tx in np.arange(-.20,.201,.025):
  for ty in np.arange(-.20,.201,.025):
   median,mean=boundary_error(tx,ty,math.radians(theta_deg))
   score=mean+.25*median
   if best is None or score<best[0]:best=(score,tx,ty,theta_deg,median,mean)
print("LANDMARK REGISTRATION:",best,"identity distance",boundary_error(0,0,0))
if abs(best[1])>.16 or abs(best[2])>.16 or abs(best[3])>5 or best[5]>.10:
 raise RuntimeError("Cannot justify physics-world to map rigid frame registration")
# Note 3x3 equal-pattern ambiguity. Search is local around the expected corresponding reference frame.
transform={"world_to_map_dx_m":round(float(best[1]),4),"world_to_map_dy_m":round(float(best[2]),4),
           "world_to_map_yaw_deg":round(float(best[3]),3),
           "boundary_mean_nearest_occupied_cell_m":round(float(best[5]),4),
           "boundary_median_nearest_occupied_cell_m":round(float(best[4]),4),
           "map_resolution_m":RES,
           "landmark_count":len(centers),
           "method":"local geometry registration: nine cylinder collision boundaries (SDF) against map occupancy cells; does not use AMCL/robot pose",
           "uncertainty":"resolution 0.05 m; local fit window +/-0.2m, +/-5deg; geometry/template approximations; NOT globally unique without more landmarks"}
def get_msgs(name,topic):
 f=list((LOG/name).glob("*.db3"));assert len(f)==1,(name,f)
 c=sqlite3.connect(str(f[0]))
 rows={n:(id,t) for id,n,t in c.execute("SELECT id,name,type FROM topics")}
 tid,typ=rows[topic];mt=get_message(typ)
 out=[deserialize_message(blob,mt) for t,blob in c.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))]
 c.close();return out
def stamps(m):
 return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
results={}
fig,ax=plt.subplots(figsize=(9,6))
for cond,run in [("A","phase5_gt_A1_rosbag"),("B","phase5_gt_B2_rosbag")]:
 gt=get_msgs(run,"/task2/ground_truth");amcl=get_msgs(run,"/amcl_pose")
 if len(gt)<80 or len(amcl)<3:raise RuntimeError("Insufficient GT or AMCL messages")
 assert all(m.header.frame_id=="world" and m.child_frame_id=="base_footprint" for m in gt)
 assert all(m.header.frame_id=="map" for m in amcl)
 tt=np.array([stamps(m) for m in gt]);xx=np.array([m.pose.pose.position.x for m in gt]);yy=np.array([m.pose.pose.position.y for m in gt]);yaw_gt=np.unwrap([yaw(m.pose.pose.orientation) for m in gt])
 if not np.all(np.diff(tt)>0):raise RuntimeError("P3D GT timestamps not strictly increasing")
 theta=math.radians(transform["world_to_map_yaw_deg"]);c,s=math.cos(theta),math.sin(theta)
 map_x=c*xx-s*yy+best[1];map_y=s*xx+c*yy+best[2]
 errs=[];yw=[];offsets=[];used=[];est=[]
 for pose in amcl:
  t=stamps(pose)
  if t<tt[0] or t>tt[-1]:continue
  k=int(np.searchsorted(tt,t))
  gap=min(abs(tt[max(0,k-1)]-t),abs(tt[min(k,len(tt)-1)]-t))
  if gap>.075:continue
  x=np.interp(t,tt,map_x);y=np.interp(t,tt,map_y)
  ex=pose.pose.pose.position.x;ey=pose.pose.pose.position.y
  err=math.hypot(ex-x,ey-y)
  dyaw=(yaw(pose.pose.pose.orientation)-np.interp(t,tt,yaw_gt)-theta+math.pi)%(2*math.pi)-math.pi
  errs.append(err);yw.append(abs(dyaw));offsets.append(gap);used.append((x,y));est.append((ex,ey))
 if len(errs)<3:raise RuntimeError("Too few time-matched AMCL poses")
 results[cond]={"ground_truth_count":len(gt),"amcl_total":len(amcl),"time_aligned_pairs":len(errs),
                "gt_timestamp_span_s":round(tt[-1]-tt[0],3),
                "max_nearest_gt_stamp_gap_s":round(max(offsets),5),
                "map_xy_rmse_m":round(math.sqrt(float(np.mean(np.square(errs)))),5),
                "map_xy_mae_m":round(float(np.mean(errs)),5),
                "map_xy_max_m":round(float(max(errs)),5),
                "yaw_mean_abs_error_deg":round(math.degrees(float(np.mean(yw))),3),
                "matched_time_error_m":[round(x,5) for x in errs],
                "validity":"simulation ground truth, header.stamp interpolation, static map landmark calibration; one run per condition"}
 print(cond,json.dumps(results[cond],indent=2))
 ax.plot(map_x,map_y,lw=1.5,label=f"{cond} P3D GT (map aligned)")
 ax.scatter([p[0] for p in est],[p[1] for p in est],s=35,label=f"{cond} AMCL matched")
ax.set(title="Phase 5: physics GT vs timestamp-matched AMCL observations",xlabel="map X [m]",ylabel="map Y [m]")
ax.set_aspect("equal",adjustable="datalim");ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.text(.1,.01,"Map/world registration calibrated from nine static cylinder boundaries; only one run per condition.",fontsize=8)
fig.savefig(EVID/"phase5_GTA_GTB_timestamp_aligned_localization.png",dpi=180,bbox_inches="tight")
plt.close(fig)
summary={"calibration":transform,"results":results,"warning":"One instrumented run each. Do not infer EKF superiority. RMSE conditional on landmark registration resolution and AMCL stamp semantics."}
(EVID/"phase5_ground_truth_alignment_and_errors.json").write_text(json.dumps(summary,indent=2)+"\n")
print("SAVED",EVID/"phase5_ground_truth_alignment_and_errors.json")
