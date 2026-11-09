#!/usr/bin/env python3
"""Controlled synthetic-scan stop-zone test of actual Humble nav2_collision_monitor.
No Gazebo or hardware motor drivers. Compare input Twist vs output Twist.
"""
from pathlib import Path
import json,math,time
from datetime import datetime,timezone
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

OUT=Path(__file__).resolve().parents[1]/"evidence/phase5_stopzone_isolated_test_v2.json"
if OUT.exists():
    raise SystemExit("Existing test JSON; refuse overwrite")
rclpy.init()
node=rclpy.create_node("phase5_stopzone_synthetic_evaluator")
scan_pub=node.create_publisher(LaserScan,"/scan",qos_profile_sensor_data)
cmd_pub=node.create_publisher(Twist,"/cmd_vel_pre_collision",10)
out_log=[]
phase=["initial"]
start=time.monotonic()

def cb(msg):
    out_log.append({"monotonic_s":round(time.monotonic()-start,3),"phase":phase[0],
                    "lin":round(msg.linear.x,5),"ang":round(msg.angular.z,5)})
sub=node.create_subscription(Twist,"/cmd_vel",cb,10)
report={"created_utc":datetime.now(timezone.utc).isoformat(),"scenario":"synthetic LaserScan into real nav2_collision_monitor, no Gazebo/hardware",
        "command_linear_m_s":.12,"hazard_front_range_m":.30,"stop_polygon_x":[.10,.50],"stop_polygon_y":[-.28,.28],
        "phases":{}}
try:
    def send_scan(obstacle):
        scan=LaserScan()
        scan.header.stamp=node.get_clock().now().to_msg()
        scan.header.frame_id="base_scan"
        scan.angle_min=-math.pi
        scan.angle_increment=2*math.pi/360
        scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.time_increment=0.0
        scan.scan_time=0.10
        scan.range_min=0.12
        scan.range_max=6.0
        scan.ranges=[float("inf")]*360
        if obstacle:
            for k in range(360):
                if abs(scan.angle_min+k*scan.angle_increment)<math.radians(12):
                    scan.ranges[k]=.30
        scan_pub.publish(scan)
    def run_phase(label,obstacle,seconds,settle=.45):
        phase[0]=label
        begun=time.monotonic();n_in=0
        while time.monotonic()-begun<seconds:
            send_scan(obstacle)
            v=Twist();v.linear.x=.12
            cmd_pub.publish(v);n_in+=1
            for j in range(4):rclpy.spin_once(node,timeout_sec=.025)
        vals=[x for x in out_log if x["phase"]==label and x["monotonic_s"]>begun-start+settle]
        report["phases"][label]={"input_count":n_in,"output_count_after_settle":len(vals),
                                  "positive_output_count":sum(x["lin"]>.09 for x in vals),
                                  "zero_output_count":sum(abs(x["lin"])<.005 and abs(x["ang"])<.005 for x in vals),
                                  "output_values":list(sorted(set(x["lin"] for x in vals)))[:10]}
        print("PHASE_RESULT",label,report["phases"][label],flush=True)
    # Startup margin for static TF, scan subscription and lifecycle auto-activation.
    begin=time.monotonic()
    while time.monotonic()-begin<3:
        send_scan(False)
        rclpy.spin_once(node,timeout_sec=.05)
    run_phase("clear_before",False,2.2)
    run_phase("hazard_in_stop_zone",True,2.2)
    run_phase("clear_after",False,2.5)
    p=report["phases"]
    a=p["clear_before"];b=p["hazard_in_stop_zone"];c=p["clear_after"]
    report["status"]="PASS" if a["positive_output_count"]>=5 and b["zero_output_count"]>=4 and b["positive_output_count"]<=1 and c["positive_output_count"]>=5 else "FAIL"
    report["first_output_samples"]=out_log[:8]
    report["last_output_samples"]=out_log[-8:]
    print("SAFETY_SOFTWARE_VERDICT",report["status"],flush=True)
finally:
    OUT.write_text(json.dumps(report,indent=2)+"\n")
    print("SAVED",OUT,flush=True)
    node.destroy_node()
    rclpy.shutdown()
if report["status"]!="PASS":raise SystemExit(1)
