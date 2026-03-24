#!/usr/bin/env python3
"""
Puck TF broadcaster node.

Maintains odom -> puck_<id> TF frames for all confirmed pucks.

On /puck/registry: replaces the full known-puck dict and immediately broadcasts
                   all frames. Because the topic is latched, a late-joining node
                   will receive the last registry on subscribe.
On timer:          re-broadcasts all known frames at ~10 Hz so that TF consumers
                   don't time out between registry updates.

Subscribes:
  /puck/registry  (foraging_msgs/PuckRegistry)

Broadcasts TF:
  odom -> puck_<id>  for every confirmed puck
"""

import rospy

from foraging_msgs.msg import PuckRegistry

import tf2_ros
from geometry_msgs.msg import TransformStamped

# ─── Parameters ──────────────────────────────────────────────────────────────
FIXED_FRAME    = "odom"
BROADCAST_RATE = 10.0   # Hz

# ─── State ───────────────────────────────────────────────────────────────────
# Maps puck_id -> {"x": float, "y": float}
confirmed_pucks: dict = {}

tf_broadcaster = None


# ─── Helpers ─────────────────────────────────────────────────────────────────

def make_transform(puck_id: int, x: float, y: float,
                   stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp    = stamp
    t.header.frame_id = FIXED_FRAME
    t.child_frame_id  = f"puck_{puck_id}"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0   # projected to ground plane

    t.transform.rotation.w = 1.0      # identity rotation

    return t


# ─── Callback ────────────────────────────────────────────────────────────────

def callback_registry(msg: PuckRegistry):
    """Replace the full puck dict from the registry and broadcast immediately."""
    global confirmed_pucks
    confirmed_pucks = {
        int(p.id): {"x": float(p.x), "y": float(p.y)}
        for p in msg.pucks
    }
    if not confirmed_pucks:
        return
    now = rospy.Time.now()
    tf_broadcaster.sendTransform([
        make_transform(pid, data["x"], data["y"], now)
        for pid, data in confirmed_pucks.items()
    ])
    rospy.logdebug("Registry update: %d confirmed puck(s)", len(confirmed_pucks))


# ─── Timer broadcast ─────────────────────────────────────────────────────────

def timer_broadcast(_event):
    """Re-broadcast all frames so TF consumers don't time out."""
    if not confirmed_pucks:
        return
    now = rospy.Time.now()
    tf_broadcaster.sendTransform([
        make_transform(pid, data["x"], data["y"], now)
        for pid, data in confirmed_pucks.items()
    ])


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    global tf_broadcaster

    rospy.init_node("puck_tf_broadcaster_node")

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.Subscriber("/puck/registry", PuckRegistry,
                     callback_registry, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / BROADCAST_RATE), timer_broadcast)

    rospy.loginfo("puck_tf_broadcaster_node started — broadcasting at %.0f Hz",
                  BROADCAST_RATE)
    rospy.spin()


if __name__ == "__main__":
    main()