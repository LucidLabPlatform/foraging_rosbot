#!/usr/bin/env python3
"""
ArUco marker localization node.

- Backprojects /aruco/detected (pixel + depth) into 3D map frame
- Clusters observations by marker_id; each unique marker_id is its own track
- Publishes /aruco/confirmed ONCE when a marker is first confirmed
- Publishes /aruco/registry (ArucoRegistry) on every confirmation — full list,
  never shrinks (confirmed markers are never pruned)
- Does NOT broadcast TF frames — handled by 4_pub_aruco_tf.py

Subscribes:
  /aruco/detected            (foraging_msgs/ArucoDetected)
  /camera/depth/camera_info  (sensor_msgs/CameraInfo)

Publishes:
  /aruco/confirmed           (foraging_msgs/ArucoConfirmed)  -- once per new unique marker
  /aruco/registry            (foraging_msgs/ArucoRegistry)   -- full confirmed list, latched
"""

import rospy
import math

from sensor_msgs.msg import CameraInfo
from foraging_msgs.msg import ArucoConfirmed, ArucoDetected, ArucoRegistry

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# ─── Camera intrinsics (populated on first CameraInfo message) ────────────────
fx = fy = cx = cy = None

# ─── TF ───────────────────────────────────────────────────────────────────────
tf_buffer   = None
tf_listener = None

FIXED_FRAME = "map"
TF_TIMEOUT  = rospy.Duration(0.15)

# ─── Clustering / confirmation params ────────────────────────────────────────
CONFIRM_HITS  = 2             # observations before a marker is "confirmed"
STALE_TIMEOUT = rospy.Duration(5.0)
EWMA_ALPHA    = 0.5            # weight for exponential moving average

# ─── Marker registry ──────────────────────────────────────────────────────────
# Each entry:
#   {
#     "id":        int,    # internal track id
#     "marker_id": int,    # ArUco marker id
#     "hits":      int,
#     "confirmed": bool,
#     "x":         float,
#     "y":         float,
#     "z":         float,
#     "last_seen": rospy.Time,
#   }
markers  = []
next_id  = 0
pub          = None
pub_registry = None


# ─── Helpers ──────────────────────────────────────────────────────────────────

def dist3(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def prune_stale(now=None):
    """Remove unconfirmed candidate tracks that haven't been updated recently."""
    global markers
    if now is None:
        now = rospy.Time.now()
    before = len(markers)
    markers = [m for m in markers if
               m["confirmed"] or
               (now - m.get("last_seen", rospy.Time(0))) <= STALE_TIMEOUT]
    if len(markers) != before:
        rospy.logdebug("Pruned %d stale candidate(s)", before - len(markers))


def _make_confirmed_msg(m) -> ArucoConfirmed:
    out = ArucoConfirmed()
    out.id        = m["id"]
    out.marker_id = m["marker_id"]
    out.status    = 1
    out.x         = float(m["x"])
    out.y         = float(m["y"])
    out.z         = float(m["z"])
    return out


def publish_registry():
    reg = ArucoRegistry()
    reg.markers = [_make_confirmed_msg(m) for m in markers if m["confirmed"]]
    pub_registry.publish(reg)


# ─── Camera info ──────────────────────────────────────────────────────────────

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


# ─── Backprojection ───────────────────────────────────────────────────────────

def backproject(u, v, depth_m):
    if fx is None or depth_m <= 0.0:
        return None
    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m
    return (X, Y, Z)


def transform_to_map(X, Y, Z, stamp, camera_frame):
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

    return None


# ─── Main detection callback ──────────────────────────────────────────────────

def callback_detected(msg: ArucoDetected):
    global next_id

    if fx is None:
        rospy.logwarn_throttle(2.0, "Waiting for camera intrinsics...")
        return

    prune_stale()

    cam_frame = msg.header.frame_id
    if not cam_frame:
        rospy.logwarn_throttle(2.0, "ArucoDetected missing frame_id")
        return

    cam_pt = backproject(msg.center_x, msg.center_y, msg.distance)
    if cam_pt is None:
        return

    map_pt = transform_to_map(
        cam_pt[0], cam_pt[1], cam_pt[2], msg.header.stamp, cam_frame)
    if map_pt is None:
        return

    marker_id = int(msg.marker_id)

    # Find existing track for this marker_id
    track = next((m for m in markers if m["marker_id"] == marker_id), None)

    if track is None:
        # New marker — start a candidate track
        markers.append({
            "id":        next_id,
            "marker_id": marker_id,
            "hits":      1,
            "confirmed": False,
            "x":         map_pt[0],
            "y":         map_pt[1],
            "z":         map_pt[2],
            "last_seen": rospy.Time.now(),
        })
        next_id += 1
        return

    # Merge observation into existing track via EWMA
    track["x"] = EWMA_ALPHA * map_pt[0] + (1 - EWMA_ALPHA) * track["x"]
    track["y"] = EWMA_ALPHA * map_pt[1] + (1 - EWMA_ALPHA) * track["y"]
    track["z"] = EWMA_ALPHA * map_pt[2] + (1 - EWMA_ALPHA) * track["z"]
    track["hits"] += 1
    track["last_seen"] = rospy.Time.now()

    rospy.logdebug("ArUco marker_id=%d | hits=%d | pos=(%.2f, %.2f, %.2f)",
                   marker_id, track["hits"], track["x"], track["y"], track["z"])

    if not track["confirmed"] and track["hits"] >= CONFIRM_HITS:
        track["confirmed"] = True

        out = _make_confirmed_msg(track)
        pub.publish(out)
        rospy.loginfo(
            "ArUco CONFIRMED: marker_id=%d pos=(%.3f, %.3f, %.3f)",
            out.marker_id, out.x, out.y, out.z)

        publish_registry()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    global pub, pub_registry, tf_buffer, tf_listener

    rospy.init_node("aruco_localization_node")

    tf_buffer   = tf2_ros.Buffer(rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub          = rospy.Publisher("/aruco/confirmed", ArucoConfirmed, queue_size=10)
    pub_registry = rospy.Publisher("/aruco/registry",  ArucoRegistry,  queue_size=1,
                                   latch=True)

    rospy.Subscriber("/camera/depth/camera_info", CameraInfo,
                     callback_info, queue_size=1)
    rospy.Subscriber("/aruco/detected", ArucoDetected,
                     callback_detected, queue_size=50)

    rospy.loginfo("aruco_localization_node started — CONFIRM_HITS=%d", CONFIRM_HITS)
    rospy.spin()


if __name__ == "__main__":
    main()

