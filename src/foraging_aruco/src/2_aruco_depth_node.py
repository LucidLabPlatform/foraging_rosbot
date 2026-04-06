#!/usr/bin/env python3
"""
Combines /aruco/detected/raw with depth data.

Subscribes:
  /aruco/detected/raw                      (foraging_msgs/RawArucoDetected)
  /camera/depth/image_2fps/compressedDepth (sensor_msgs/CompressedImage)

Publishes:
  /aruco/detected                          (foraging_msgs/ArucoDetected)
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawArucoDetected, ArucoDetected

DEBUG = False

pending_markers = []   # RawArucoDetected msgs waiting for a depth frame
publisher_aruco = None


def callback_raw_aruco(msg: RawArucoDetected):
    pending_markers.append(msg)


def callback_depth(msg: CompressedImage):
    global pending_markers

    if not pending_markers:
        return

    # Decode compressedDepth (16-byte header + PNG payload)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    depth_img = cv2.imdecode(buf[12:], cv2.IMREAD_UNCHANGED)  # uint16, mm
    if depth_img is None:
        return
    depth_img = depth_img.astype(np.float32)
    h, w = depth_img.shape

    to_process  = pending_markers[:]
    pending_markers.clear()

    if DEBUG:
        display = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        display = display.astype(np.uint8)
        display = cv2.applyColorMap(display, cv2.COLORMAP_JET)

    for marker in to_process:
        cx = int(np.clip(marker.center_x, 0, w - 1))
        cy = int(np.clip(marker.center_y, 0, h - 1))

        # 5px patch median — robust to depth holes
        PATCH = 5
        patch = depth_img[max(0, cy - PATCH):min(h, cy + PATCH),
                          max(0, cx - PATCH):min(w, cx + PATCH)]
        valid = patch[patch > 0]
        if valid.size == 0:
            rospy.logwarn_throttle(2, f"Depth hole at ({cx},{cy}) for marker {marker.marker_id} — skipping")
            continue

        distance_mm = float(np.median(valid))
        if not (100 < distance_mm < 5000):
            rospy.logdebug(f"Depth out of range {distance_mm:.0f}mm for marker {marker.marker_id} — skipping")
            continue

        distance_m = distance_mm / 1000.0
        rospy.logdebug(f"ArUco id={marker.marker_id} at ({cx},{cy}) → {distance_m:.3f}m")

        out = ArucoDetected()
        out.header    = marker.header
        out.center_x  = float(cx)
        out.center_y  = float(cy)
        out.distance  = distance_m
        out.marker_id = marker.marker_id
        publisher_aruco.publish(out)

        if DEBUG:
            cv2.circle(display, (cx, cy), 6, (0, 255, 0), -1)
            cv2.putText(display, f"id={marker.marker_id} {distance_m:.2f}m",
                        (cx + 8, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if DEBUG:
        cv2.imshow("aruco_depth", display)
        cv2.waitKey(1)


def main():
    global publisher_aruco
    rospy.init_node("aruco_depth_node")

    publisher_aruco = rospy.Publisher("/aruco/detected", ArucoDetected, queue_size=10)
    rospy.Subscriber("/camera/depth/image_2fps/compressedDepth", CompressedImage,
                     callback_depth, queue_size=1)
    rospy.Subscriber("/aruco/detected/raw", RawArucoDetected,
                     callback_raw_aruco, queue_size=10)

    rospy.loginfo("aruco_depth_node started")
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
