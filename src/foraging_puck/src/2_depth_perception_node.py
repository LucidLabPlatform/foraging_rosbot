#!/usr/bin/env python3
"""
Get /puck/detected/raw:
 - header (copied from color image msg)
 - center_x (image pixel coordinates, full image space)
 - center_y (image pixel coordinates, full image space)
 - color (0=unknown,1=red,2=green,3=blue)
Publish /puck/detected with:
 - std_msgs/Header header
 - float32 center_x
 - float32 center_y
 - float32 distance   # estimated distance from depth image
 - int32 color        # 0=unknown  1=red  2=green  3=blue
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawPuckDetected, PuckDetected

DEBUG = False
CROP_TOP_FRACTION    = 0.5   # Must match perception_node.py
DEPTH_CLUSTER_TOL_MM  = 150  # Readings within 150mm are considered the same surface
MIN_CLUSTER_SAMPLES   = 5    # Minimum readings in dominant cluster to trust the depth
MIN_VALID_DEPTH_RATIO = 0.5  # Skip puck if fewer than 50% of circle pixels have valid depth

pending_pucks = []
publisher_puck = None
COLOR_BGR = {1: (0, 0, 255), 2: (0, 255, 0), 3: (255, 0, 0)}


def callback_raw_puck(msg: RawPuckDetected):
    pending_pucks.append(msg)


def callback_depth(msg: CompressedImage):
    global pending_pucks
    if not pending_pucks:
        return

    # Decode depth once per frame
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    depth_img = cv2.imdecode(buf[12:], cv2.IMREAD_UNCHANGED)   # uint16, mm
    if depth_img is None:
        return

    depth_img = depth_img.astype(np.float32)
    full_h, w = depth_img.shape

    # --- Crop: keep only the bottom (1 - CROP_TOP_FRACTION) of the depth image ---
    crop_y = int(full_h * CROP_TOP_FRACTION)
    depth_cropped = depth_img[crop_y:, :]
    cropped_h = depth_cropped.shape[0]
    # ------------------------------------------------------------------------------

    to_process  = pending_pucks
    pending_pucks = []

    if DEBUG:
        display = cv2.normalize(depth_cropped, None, 0, 255, cv2.NORM_MINMAX)
        display = display.astype(np.uint8)
        display = cv2.applyColorMap(display, cv2.COLORMAP_JET)

    for puck in to_process:
        # Convert center from full-image space to cropped-image space
        cx = int(np.clip(puck.center_x, 0, w - 1))
        cy_full = int(puck.center_y)
        cy_cropped = cy_full - crop_y

        if cy_cropped < 0 or cy_cropped >= cropped_h:
            rospy.logwarn_throttle(2, "Puck center is above crop line — skipping")
            continue

        # Build mask from contour
        if len(puck.contour_x) < 3:
            rospy.logwarn_throttle(2, "Puck contour too small — skipping")
            continue
        mask = np.zeros((cropped_h, w), dtype=np.uint8)
        pts = np.column_stack((puck.contour_x, puck.contour_y)).astype(np.int32)
        cv2.fillPoly(mask, [pts], 255)

        masked_depth = depth_cropped.copy()
        masked_depth[mask == 0] = 0
        valid = masked_depth[masked_depth > 0].flatten()

        total_mask_pixels = np.count_nonzero(mask)
        if len(valid) < total_mask_pixels * MIN_VALID_DEPTH_RATIO:
            rospy.logdebug(f"Too many invalid depth pixels ({len(valid)}/{total_mask_pixels}) — skipping")
            continue

        # Find the largest cluster of similar depths
        valid_sorted = np.sort(valid)
        best_cluster = np.array([])
        for i in range(len(valid_sorted)):
            cluster = valid_sorted[
                (valid_sorted >= valid_sorted[i]) &
                (valid_sorted <= valid_sorted[i] + DEPTH_CLUSTER_TOL_MM)
            ]
            if len(cluster) > len(best_cluster):
                best_cluster = cluster

        if len(best_cluster) < MIN_CLUSTER_SAMPLES:
            rospy.logwarn_throttle(2, f"No dominant depth surface ({len(best_cluster)} samples) — skipping")
            continue

        distance_mm = float(np.mean(best_cluster))

        if not (100 < distance_mm < 2000):
            rospy.logdebug(f"Depth out of range {distance_mm:.0f}mm — skipping")
            continue

        distance_m = distance_mm / 1000.0
        # rospy.loginfo(f"Puck color={puck.color} at ({cx},{cy_full}) → {distance_m:.3f}m")

        out          = PuckDetected()
        out.header   = puck.header
        out.center_x = float(cx)
        out.center_y = float(cy_full)   # publish in full-image space
        out.distance = distance_m
        out.color    = puck.color
        publisher_puck.publish(out)

        if DEBUG:
            color_bgr = COLOR_BGR.get(puck.color, (255, 255, 255))
            cv2.polylines(display, [pts], True, color_bgr, 1)
            cv2.circle(display, (cx, cy_cropped), 6, color_bgr, -1)      # center dot
            cv2.putText(display, f"{distance_m:.2f}m", (cx + 8, cy_cropped - 6),
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