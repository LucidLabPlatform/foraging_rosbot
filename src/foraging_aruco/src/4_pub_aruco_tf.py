#!/usr/bin/env python3
"""
ArUco TF broadcaster node.

Maintains odom -> aruco_<marker_id> TF frames for all confirmed markers.

On /aruco/registry: replaces the full known-marker dict and immediately
                    broadcasts all frames. Topic is latched so late-joiners
                    receive the last registry on subscribe.
On timer:           re-broadcasts all known frames at ~10 Hz so TF consumers
                    don't time out between registry updates.

Subscribes:
  /aruco/registry  (foraging_msgs/ArucoRegistry)

Broadcasts TF:
  odom -> aruco_<marker_id>  for every confirmed marker
"""

import rospy

from foraging_msgs.msg import ArucoRegistry

import tf2_ros
from geometry_msgs.msg import TransformStamped

# ─── Parameters ───────────────────────────────────────────────────────────────
FIXED_FRAME    = "odom"
BROADCAST_RATE = 10.0   # Hz

# ─── State ────────────────────────────────────────────────────────────────────
# Maps marker_id -> {"x": float, "y": float}
confirmed_markers: dict = {}

tf_broadcaster = None


# ─── Helpers ──────────────────────────────────────────────────────────────────

def make_transform(marker_id: int, x: float, y: float,
                   stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp    = stamp
    t.header.frame_id = FIXED_FRAME
    t.child_frame_id  = f"aruco_{marker_id}"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0   # projected to ground plane

    t.transform.rotation.w = 1.0      # identity rotation

    return t


# ─── Callback ─────────────────────────────────────────────────────────────────

def callback_registry(msg: ArucoRegistry):
    """Replace the full marker dict from the registry and broadcast immediately."""
    global confirmed_markers
    confirmed_markers = {
        int(m.marker_id): {"x": float(m.x), "y": float(m.y)}
        for m in msg.markers
    }
    if not confirmed_markers:
        return
    now = rospy.Time.now()
    tf_broadcaster.sendTransform([
        make_transform(mid, data["x"], data["y"], now)
        for mid, data in confirmed_markers.items()
    ])
    rospy.logdebug("Registry update: %d confirmed marker(s)", len(confirmed_markers))


# ─── Timer broadcast ──────────────────────────────────────────────────────────

def timer_broadcast(_event):
    """Re-broadcast all frames so TF consumers don't time out."""
    if not confirmed_markers:
        return
    now = rospy.Time.now()
    tf_broadcaster.sendTransform([
        make_transform(mid, data["x"], data["y"], now)
        for mid, data in confirmed_markers.items()
    ])


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    global tf_broadcaster

    rospy.init_node("aruco_tf_broadcaster_node")

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.Subscriber("/aruco/registry", ArucoRegistry,
                     callback_registry, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / BROADCAST_RATE), timer_broadcast)

    rospy.loginfo("aruco_tf_broadcaster_node started — broadcasting at %.0f Hz",
                  BROADCAST_RATE)
    rospy.spin()


if __name__ == "__main__":
    main()
