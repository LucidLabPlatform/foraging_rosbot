#!/usr/bin/env python3

"""
Get /puck/detected/raw:
 - header (copied from color image msg)
 - center_x (image pixel coordinates)
 - center_y (image pixel coordinates)
 - color (0=red,1=green,2=blue)
Publish /puck/detected with:
 - std_msgs/Header header     # timestamp + camera frame_id
 - float32 center_x           # center y position of puck in image
 - float32 center_y           # center y position of puck in image
 - float32 distance           # estimated distance from depth image
 - int32 color                # 0=red  1=green  2=blue
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawPuckDetected, PuckDetected

DEBUG = True

pending_pucks = []   # list of RawPuckDetected msgs waiting for depth
publisher_puck = None

COLOR_BGR = {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 255, 255)}


def callback_raw_puck(msg: RawPuckDetected):
    pending_pucks.append(msg)


def callback_depth(msg: CompressedImage):
    global pending_pucks

    if not pending_pucks:
        return

    # Decode depth once per frame
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    depth_img = cv2.imdecode(buf[12:], cv2.IMREAD_UNCHANGED)  # uint16, mm
    if depth_img is None:
        return
    depth_img = depth_img.astype(np.float32)
    h, w = depth_img.shape

    # Grab all detections accumulated since last depth frame, then reset
    to_process  = pending_pucks
    pending_pucks = []

    # Debug display — built once, dots added per puck
    if DEBUG:
        display = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        display = display.astype(np.uint8)
        display = cv2.applyColorMap(display, cv2.COLORMAP_JET)

    for puck in to_process:
        cx = int(np.clip(puck.center_x, 0, w - 1))
        cy = int(np.clip(puck.center_y, 0, h - 1))

        # 5px patch median — robust to depth holes
        PATCH = 5
        patch = depth_img[max(0, cy-PATCH):min(h, cy+PATCH),
                          max(0, cx-PATCH):min(w, cx+PATCH)]
        valid = patch[patch > 0]
        #if valid.size == 0:
        #    rospy.logwarn_throttle(2, f"Depth hole at ({cx},{cy}) — skipping")
        #    continue

        distance_mm = float(np.median(valid))
        if not (100 < distance_mm < 2000):
            rospy.logwarn_throttle(2, f"Depth out of range {distance_mm:.0f}mm — skipping")
            continue
        else:
            distance_m = distance_mm / 1000.0
            rospy.loginfo(f"Puck color={puck.color} at ({cx},{cy}) → {distance_m:.3f}m")

            # Publish
            out = PuckDetected()
            out.header   = puck.header
            out.center_x = float(cx)
            out.center_y = float(cy)
            out.distance = distance_m
            out.color    = puck.color
            publisher_puck.publish(out)

        if DEBUG:
            color_bgr = COLOR_BGR.get(puck.color, (255, 255, 255))
            cv2.circle(display, (cx, cy), 6, color_bgr, -1)
            cv2.putText(display, f"{distance_m:.2f}m", (cx + 8, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

    if DEBUG:
        cv2.imshow("depth", display)
        cv2.waitKey(1)


def main():
    global publisher_puck
    rospy.init_node("puck_depth_node")

    publisher_puck = rospy.Publisher('/puck/detected', PuckDetected, queue_size=10)
    rospy.Subscriber("/camera/depth/image_2fps/compressedDepth", CompressedImage, callback_depth, queue_size=1)
    rospy.Subscriber("/puck/detected/raw", RawPuckDetected, callback_raw_puck, queue_size=10)

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()