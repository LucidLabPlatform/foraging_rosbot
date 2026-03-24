#!/usr/bin/env python3
"""
Converts /puck/registry into a sensor_msgs/PointCloud2 that move_base's
obstacle_layer consumes as a virtual sensor.

Key fixes vs previous version:
  - Points published at z=0.15 (clear of the min_obstacle_height=0 boundary)
  - Stamp is slightly in the past to avoid TF extrapolation errors
  - Diagnostic counters printed every 5 s so you can see it's alive
  - Cloud is published on every spin tick, not only when registry changes
"""

import rospy
import struct
import math
from foraging_msgs.msg import PuckRegistry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

PUCK_RADIUS   = 0.20   # metres — inflate this if the robot still clips pucks
POINT_SPACING = 0.01   # grid resolution inside the disc
POINT_Z       = 0.0   # must be > min_obstacle_height (0.0) and < max_obstacle_height (0.5)


def make_disc(cx, cy, cz=POINT_Z):
    """Solid disc of points centred on (cx, cy)."""
    pts = [(cx, cy, cz)]
    steps = max(1, int(PUCK_RADIUS / POINT_SPACING))
    for r_step in range(1, steps + 1):
        r = r_step * POINT_SPACING
        n_pts = max(8, int(2 * math.pi * r / POINT_SPACING))
        for i in range(n_pts):
            a = 2 * math.pi * i / n_pts
            pts.append((cx + r * math.cos(a),
                        cy + r * math.sin(a),
                        cz))
    return pts


def build_cloud(frame_id, points):
    """Pack (x,y,z) tuples into a PointCloud2."""
    h = Header()
    # Stamp slightly in the past — prevents TF "would extrapolate into future" warnings
    h.stamp    = rospy.Time.now() - rospy.Duration(0.1)
    h.frame_id = frame_id

    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]

    data = bytearray()
    for (x, y, z) in points:
        data += struct.pack('fff', x, y, z)

    cloud              = PointCloud2()
    cloud.header       = h
    cloud.height       = 1
    cloud.width        = len(points) if points else 0
    cloud.fields       = fields
    cloud.is_bigendian = False
    cloud.point_step   = 12
    cloud.row_step     = 12 * cloud.width
    cloud.data         = bytes(data)
    cloud.is_dense     = True
    return cloud


class PuckObstaclePublisher:

    def __init__(self):
        rospy.init_node('puck_obstacle_publisher')

        self.frame_id        = 'map'
        self.rate_hz         = 5.0
        self.latest_registry = None
        self.pub_count       = 0
        self.last_log        = rospy.Time.now()

        self.pub = rospy.Publisher('/puck/as_obstacles', PointCloud2,
                                   queue_size=1, latch=False)
        rospy.Subscriber('/puck/registry', PuckRegistry, self._cb)
        rospy.loginfo('[puck_obstacles] node ready')

    def _cb(self, msg):
        self.latest_registry = msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():

            if self.latest_registry is None:
                # publish empty cloud so the layer doesn't stall waiting
                self.pub.publish(build_cloud(self.frame_id, []))
                rate.sleep()
                continue

            all_pts = []
            n_pucks = 0
            for puck in self.latest_registry.pucks:
                all_pts.extend(make_disc(puck.x, puck.y))
                n_pucks += 1

            self.pub.publish(build_cloud(self.frame_id, all_pts))
            self.pub_count += 1

            # Diagnostic log every 5 s
            now = rospy.Time.now()
            if (now - self.last_log).to_sec() >= 5.0:
                rospy.loginfo(
                    '[puck_obstacles] published %d msgs | '
                    'last cloud: %d pucks, %d points',
                    self.pub_count, n_pucks, len(all_pts))
                self.last_log = now

            rate.sleep()


if __name__ == '__main__':
    try:
        PuckObstaclePublisher().spin()
    except rospy.ROSInterruptException:
        pass