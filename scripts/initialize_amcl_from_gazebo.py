#!/usr/bin/env python3
"""Optional controlled AMCL initialization from Gazebo world pose.

Valid only after verifying the occupancy map is aligned to Gazebo world.
Use --apply to request a runtime initial pose; modifies no files.
Not a ground-truth accuracy measurement or proof of convergence.
"""
import argparse
import math
import subprocess
import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetInitialPose

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="waffle")
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()

    out = subprocess.run(["gz", "model", "-m", args.model, "-p"],
                         capture_output=True, text=True, timeout=5, check=True)
    x, y, z, roll, pitch, yaw = map(float, out.stdout.strip().split())
    print(f"Gazebo model pose: x={x:.4f}, y={y:.4f}, yaw={yaw:.4f} rad")
    print("Assumption: Gazebo world XY axis/origin match OccupancyGrid map frame.")
    if not args.apply:
        print("DRY RUN: no ROS localization changes. Use --apply after verifying map alignment.")
        return

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = Node("group_e_initial_pose_calibrator")
    try:
        client = node.create_client(SetInitialPose, "/set_initial_pose")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/set_initial_pose service not available")
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        req = SetInitialPose.Request()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = node.get_clock().now().to_msg()
        req.pose.pose.pose.position.x = x
        req.pose.pose.pose.position.y = y
        req.pose.pose.pose.orientation.z = math.sin(yaw / 2)
        req.pose.pose.pose.orientation.w = math.cos(yaw / 2)
        # Confidence is chosen for a known simulation start, not a measured error.
        req.pose.pose.covariance[0] = 0.05 ** 2
        req.pose.pose.covariance[7] = 0.05 ** 2
        req.pose.pose.covariance[35] = math.radians(5) ** 2
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        if not future.done() or future.exception():
            raise RuntimeError(f"AMCL service failed: {future.exception() if future.done() else 'timeout'}")
        print("AMCL SetInitialPose request delivered; verify TF and AMCL response.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
