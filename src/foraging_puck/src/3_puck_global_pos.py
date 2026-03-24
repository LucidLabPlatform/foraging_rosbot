#!/usr/bin/env python3
"""
Puck localization node.

- Backprojects /puck/detected (pixel + depth) into 3D odom frame
- Clusters observations to deduplicate; assigns each unique puck a stable ID
- Publishes /puck/confirmed ONCE when a puck is first confirmed
- Publishes /puck/registry (PuckRegistry) on every confirmation — full list of
  all confirmed pucks; never shrinks (confirmed pucks are never pruned)
- Does NOT broadcast TF frames — that is handled by puck_tf_broadcaster_node.py

Subscribes:
  /puck/detected             (foraging_msgs/PuckDetected)
  /camera/depth/camera_info  (sensor_msgs/CameraInfo)

Publishes:
  /puck/confirmed            (foraging_msgs/PuckConfirmed)   -- once per new unique puck
  /puck/registry             (foraging_msgs/PuckRegistry)    -- full confirmed list, latched
"""

import rospy
import math

from sensor_msgs.msg import CameraInfo
from foraging_msgs.msg import PuckConfirmed, PuckDetected, PuckRegistry

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# ─── Camera intrinsics (populated on first CameraInfo message) ───────────────
fx = fy = cx = cy = None

# ─── TF ──────────────────────────────────────────────────────────────────────
tf_buffer   = None
tf_listener = None

FIXED_FRAME = "odom"
TF_TIMEOUT  = rospy.Duration(0.15)

# ─── Clustering / confirmation params ────────────────────────────────────────
CLUSTER_R      = 0.3   # metres — observations within this radius are the same puck
CONFIRM_HITS   = 3     # number of observations before a puck is "confirmed"
STALE_TIMEOUT  = rospy.Duration(5.0)  # drop tracks not updated within this time
EWMA_ALPHA     = 0.5   # weight for exponential moving average (0.0 to 1.0)

# ─── Puck registry ───────────────────────────────────────────────────────────
# Each entry:
#   {
#     "id":        int,
#     "color":     int,
#     "hits":      int,
#     "confirmed": bool,
#     "x":         float,  # smoothed x position
#     "y":         float,  # smoothed y position
#     "z":         float,  # smoothed z position
#     "last_seen": rospy.Time,
#   }
pucks   = []
next_id = 0
pub          = None
pub_registry = None


# ─── Helpers ─────────────────────────────────────────────────────────────────

def dist3(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def prune_stale_pucks(now=None):
    """Remove unconfirmed candidate tracks that haven't been updated recently.
    Confirmed pucks are never pruned — they remain in the registry permanently."""
    global pucks
    if now is None:
        now = rospy.Time.now()
    before = len(pucks)
    pucks = [p for p in pucks if
             p["confirmed"] or
             (now - p.get("last_seen", rospy.Time(0))) <= STALE_TIMEOUT]
    if len(pucks) != before:
        rospy.logdebug("Pruned %d stale candidate(s)", before - len(pucks))

def publish_registry():
    """Publish the full list of confirmed pucks to /puck/registry."""
    reg = PuckRegistry()
    reg.pucks = [
        _make_confirmed_msg(p)
        for p in pucks if p["confirmed"]
    ]
    pub_registry.publish(reg)


def _make_confirmed_msg(p) -> PuckConfirmed:
    out = PuckConfirmed()
    out.id     = p["id"]
    out.color  = p["color"]
    out.status = 1
    out.x      = float(p["x"])
    out.y      = float(p["y"])
    out.z      = float(p["z"])
    return out




def callback_info(msg: CameraInfo):
    global fx, fy, cx, cy
    if fx is not None:
        return
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    rospy.loginfo("Intrinsics set: fx=%.2f fy=%.2f cx=%.2f cy=%.2f frame=%s",
                  fx, fy, cx, cy, msg.header.frame_id)


# ─── Backprojection ──────────────────────────────────────────────────────────

def backproject(u, v, depth_m):
    """Pixel (u, v) + depth Z -> 3D point in camera optical frame."""
    if fx is None or depth_m <= 0.0:
        return None
    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m
    return (X, Y, Z)


def transform_to_odom(X, Y, Z, stamp, camera_frame):
    ps = PointStamped()
    ps.header.stamp    = stamp
    ps.header.frame_id = camera_frame
    ps.point.x = X
    ps.point.y = Y
    ps.point.z = Z

    for ts in (stamp, rospy.Time(0)):
        try:
            tfm = tf_buffer.lookup_transform(
                FIXED_FRAME, camera_frame, ts, TF_TIMEOUT)
            out = tf2_geometry_msgs.do_transform_point(ps, tfm)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:
            if ts == rospy.Time(0):
                rospy.logwarn_throttle(1.0, "TF failed %s->%s (latest): %s",
                                       camera_frame, FIXED_FRAME, str(e))
            else:
                rospy.logdebug("TF lookup failed at stamp %s: %s", stamp, str(e))

    return None


# ─── Main detection callback ──────────────────────────────────────────────────

def callback_detected(msg: PuckDetected):
    global next_id

    if fx is None:
        rospy.logwarn_throttle(2.0, "Waiting for camera intrinsics...")
        return

    prune_stale_pucks()

    cam_frame = msg.header.frame_id
    if not cam_frame:
        rospy.logwarn_throttle(2.0, "PuckDetected missing frame_id")
        return

    # 1. Backproject pixel + depth to camera 3D
    cam_pt = backproject(msg.center_x, msg.center_y, msg.distance)
    if cam_pt is None:
        return

    # 2. Transform to odom frame
    odom_pt = transform_to_odom(
        cam_pt[0], cam_pt[1], cam_pt[2], msg.header.stamp, cam_frame)
    if odom_pt is None:
        return

    color = int(msg.color)

    # 3. Find nearest existing cluster of the same color
    best_i = None
    best_d = float("inf")
    for i, p in enumerate(pucks):
        if p["color"] != color:
            continue
        d = dist3(odom_pt, (p["x"], p["y"], p["z"]))
        if d <= CLUSTER_R and d < best_d:
            best_d, best_i = d, i

    # 4a. No nearby cluster — start a new candidate
    if best_i is None:
        pucks.append({
            "id":        next_id,
            "color":     color,
            "hits":      1,
            "confirmed": False,
            "x":         odom_pt[0],
            "y":         odom_pt[1],
            "z":         odom_pt[2],
            "last_seen": rospy.Time.now(),
        })
        next_id += 1
        return

    # 4b. Merge observation into existing cluster via EWMA
    p = pucks[best_i]
    p["x"] = EWMA_ALPHA * odom_pt[0] + (1 - EWMA_ALPHA) * p["x"]
    p["y"] = EWMA_ALPHA * odom_pt[1] + (1 - EWMA_ALPHA) * p["y"]
    p["z"] = EWMA_ALPHA * odom_pt[2] + (1 - EWMA_ALPHA) * p["z"]
    p["hits"] += 1
    p["last_seen"] = rospy.Time.now()

    rospy.logdebug("Puck %d | color=%d | hits=%d | pos=(%.2f, %.2f, %.2f)",
                   p["id"], color, p["hits"], p["x"], p["y"], p["z"])

    # 5. First time we reach CONFIRM_HITS: publish confirmed puck + update registry
    if not p["confirmed"] and p["hits"] >= CONFIRM_HITS:
        p["confirmed"] = True

        out = _make_confirmed_msg(p)
        pub.publish(out)
        rospy.loginfo(
            "Puck CONFIRMED: id=%d color=%d pos=(%.3f, %.3f, %.3f)",
            out.id, out.color, out.x, out.y, out.z)

        publish_registry()

    # 6. Already confirmed — no further action needed (pucks are stationary)


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    global pub, pub_registry, tf_buffer, tf_listener

    rospy.init_node("puck_localization_node")

    tf_buffer   = tf2_ros.Buffer(rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub          = rospy.Publisher("/puck/confirmed", PuckConfirmed, queue_size=10)
    pub_registry = rospy.Publisher("/puck/registry",  PuckRegistry,  queue_size=1,
                                   latch=True)

    rospy.Subscriber("/camera/depth/camera_info", CameraInfo,
                     callback_info, queue_size=1)
    rospy.Subscriber("/puck/detected", PuckDetected,
                     callback_detected, queue_size=50)

    rospy.loginfo("puck_localization_node started — CONFIRM_HITS=%d CLUSTER_R=%.2fm",
                  CONFIRM_HITS, CLUSTER_R)
    rospy.spin()


if __name__ == "__main__":
    main()