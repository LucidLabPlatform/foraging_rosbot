#!/usr/bin/env python3

import rospy
import struct
import math
from foraging_msgs.msg import PuckRegistry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

PUCK_RADIUS   = 0.10   # bigger than actual puck to create hard safety margin
POINT_SPACING = 0.01
POINT_Z       = 0.0   # must be above min_obstacle_height


def make_disc(cx, cy, cz=POINT_Z):
    pts = []
    steps = max(1, int(PUCK_RADIUS / POINT_SPACING))

    for r_step in range(0, steps + 1):
        r = r_step * POINT_SPACING

        if r_step == 0:
            pts.append((cx, cy, cz))
            continue

        n_pts = max(12, int(2 * math.pi * r / POINT_SPACING))
        for i in range(n_pts):
            a = 2 * math.pi * i / n_pts
            pts.append((
                cx + r * math.cos(a),
                cy + r * math.sin(a),
                cz
            ))
    return pts


def build_cloud(frame_id, points):
    h = Header()
    h.stamp = rospy.Time.now() - rospy.Duration(0.1)
    h.frame_id = frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    data = bytearray()
    for x, y, z in points:
        data += struct.pack('fff', x, y, z)

    cloud = PointCloud2()
    cloud.header = h
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * cloud.width
    cloud.data = bytes(data)
    cloud.is_dense = True
    return cloud


class PuckObstaclePublisher:
    def __init__(self):
        rospy.init_node('puck_obstacle_publisher')

        self.frame_id = 'map'
        self.rate_hz = 5.0
        self.latest_registry = None

        self.pub = rospy.Publisher('/puck/as_obstacles', PointCloud2, queue_size=1)
        rospy.Subscriber('/puck/registry', PuckRegistry, self.cb)

        rospy.loginfo('[puck_obstacles] ready')

    def cb(self, msg):
        self.latest_registry = msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            points = []

            if self.latest_registry is not None:
                for puck in self.latest_registry.pucks:
                    if puck.status == 1:
                        points.extend(make_disc(puck.x, puck.y))

            self.pub.publish(build_cloud(self.frame_id, points))
            rate.sleep()


if __name__ == '__main__':
    try:
        PuckObstaclePublisher().spin()
    except rospy.ROSInterruptException:
        pass