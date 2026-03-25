#!/usr/bin/env python3
"""
Detects ArUco markers from the compressed color image.

Publishes foraging_msgs/RawArucoDetected with:
 - header    (copied from color image msg)
 - center_x  (image pixel coordinates)
 - center_y  (image pixel coordinates)
 - marker_id (ArUco marker ID)
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawArucoDetected

bridge = CvBridge()

DEBUG_VISUALS = False

# ArUco dictionary — change to match the markers used in the arena
ARUCO_DICT = cv2.aruco.DICT_4X4_250

aruco_dict   = cv2.aruco.Dictionary_get(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters_create()

publisher_aruco = None


def callback_color(msg: CompressedImage):
    header = msg.header
    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None:
        if DEBUG_VISUALS:
            cv2.imshow("aruco_detection", img)
            cv2.waitKey(1)
        return

    for i, marker_id in enumerate(ids.flatten()):
        # corners[i] shape: (1, 4, 2) — four corners of the marker
        pts = corners[i][0]
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))

        out = RawArucoDetected()
        out.header    = header
        out.center_x  = cx
        out.center_y  = cy
        out.marker_id = int(marker_id)
        publisher_aruco.publish(out)

        if DEBUG_VISUALS:
            cv2.aruco.drawDetectedMarkers(img, corners)
            cv2.putText(img, str(marker_id), (int(cx), int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if DEBUG_VISUALS:
        cv2.imshow("aruco_detection", img)
        cv2.waitKey(1)


def main():
    global publisher_aruco
    rospy.init_node("aruco_perception_node")
    publisher_aruco = rospy.Publisher("/aruco/detected/raw", RawArucoDetected, queue_size=10)
    rospy.Subscriber("/camera/color/image_2fps/compressed", CompressedImage,
                     callback_color, queue_size=1)
    rospy.loginfo("aruco_perception_node started — dict=DICT_4X4_250")
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
