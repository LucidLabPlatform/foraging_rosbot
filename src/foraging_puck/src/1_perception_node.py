#!/usr/bin/env python3
"""
Publishes foraging_msgs/RawPuckDetected with:
 - header (copied from color image msg)
 - center_x (image pixel coordinates, in FULL image space)
 - center_y (image pixel coordinates, in FULL image space)
 - color (0=unknown,1=red,2=green,3=blue)
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawPuckDetected

bridge = CvBridge()

# HSV bounds (tune with `python3 0_calibrate_HSV.py`)
DEBUG_VISUALS = False
CROP_TOP_FRACTION = 0.5     # Discard this fraction from the top of the image
MIN_CONTOUR_AREA = 200      # Minimum area of contour to be considered a puck
MAX_CONTOUR_AREA = 10000     # Maximum area of contour to be considered a puck
HSV_BOUNDS = {
    "red":   [(np.array([  0, 105,   10]), np.array([ 15, 255, 255])),
              (np.array([165, 70,   0]), np.array([180, 255, 255]))],
    "green": [(np.array([ 35,  40,   20]), np.array([ 80, 255, 255]))],
    "blue":  [(np.array([ 90, 160,   0]), np.array([130, 255, 255]))],
}
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def publish_detection(header, cx, cy, color_id, contour):
    global publisher_puck_center
    msg = RawPuckDetected()
    msg.header   = header
    msg.center_x = float(cx)
    msg.center_y = float(cy)
    msg.color    = int(color_id)
    pts = contour.reshape(-1, 2)
    msg.contour_x = pts[:, 0].tolist()
    msg.contour_y = pts[:, 1].tolist()
    publisher_puck_center.publish(msg)


def preprocess_mask(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    _, mask = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
    return mask


def detect_color_pucks(hsv_img, color_name, max_pucks=3):
    """
    Returns (detections, mask).
    detections is a list of (contour, cX, cY) — coordinates in the CROPPED image.
    """
    ranges = HSV_BOUNDS[color_name]
    mask = preprocess_mask(
        np.bitwise_or.reduce([cv2.inRange(hsv_img, lo, hi) for lo, hi in ranges])
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return [], mask

    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circ = 4.0 * np.pi * area / (perim * perim)
        if circ < 0.4:
            continue
        valid.append((area, c))

    valid.sort(key=lambda x: x[0], reverse=True)
    valid = valid[:max_pucks]

    detections = []
    for area, c in valid:
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        detections.append((c, cX, cY))

    return detections, mask


def callback_color(msg: CompressedImage):
    header = msg.header
    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # --- Crop: keep only the bottom (1 - CROP_TOP_FRACTION) of the image ---
    full_h = img.shape[0]
    crop_y = int(full_h * CROP_TOP_FRACTION)
    img = img[crop_y:, :]
    # ---------------------------------------------------------------------

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    for color_name, color_id in [("red", 1), ("green", 2), ("blue", 3)]:
        detections, mask = detect_color_pucks(hsv_img, color_name, max_pucks=3)

        for contour, cx, cy in detections:
            publish_detection(header, cx, cy + crop_y, color_id, contour)

            if DEBUG_VISUALS:
                cv2.drawContours(img, [contour], -1, (255, 0, 0), 2)
                cv2.circle(img, (cx, cy), 2, (0, 255, 0), -1)
                cv2.putText(img, color_name, (cx, max(0, cy - 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    if DEBUG_VISUALS:
        cv2.imshow("color_image", img)
        cv2.waitKey(1)


def main():
    global publisher_puck_center
    rospy.init_node("perception_node")
    publisher_puck_center = rospy.Publisher('/puck/detected/raw', RawPuckDetected, queue_size=4)
    rospy.Subscriber("/camera/color/image_2fps/compressed", CompressedImage, callback_color, queue_size=1)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()