#!/usr/bin/env python3
"""
Publishes foraging_msgs/RawPuckDetected with:
 - header (copied from color image msg)
 - center_x (image pixel coordinates, in FULL image space)
 - center_y (image pixel coordinates, in FULL image space)
 - color (0=red,1=green,2=blue)
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from foraging_msgs.msg import RawPuckDetected

bridge = CvBridge()

# HSV bounds (tune with `python3 0_calibrate_HSV.py`)
DEBUG_VISUALS = True
CROP_TOP_FRACTION = 0.5     # Discard this fraction from the top of the image
MIN_CONTOUR_AREA = 200      # Minimum area of contour to be considered a puck
MAX_CONTOUR_AREA = 5000     # Maximum area of contour to be considered a puck
HSV_BOUNDS = {
    "red":   (np.array([165, 105,   0]), np.array([180, 255, 255])),
    "green": (np.array([ 35, 20,   0]), np.array([ 80, 255, 255])),
    "blue":  (np.array([ 90, 160,   0]), np.array([130, 255, 255])),
}
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def publish_detection(header, cx, cy, bx, by, bw, bh, color_id):
    global publisher_puck_center
    msg_puck_center = RawPuckDetected()
    msg_puck_center.header   = header
    msg_puck_center.center_x = float(cx)
    msg_puck_center.center_y = float(cy)
    msg_puck_center.bbox_x   = int(bx)
    msg_puck_center.bbox_y   = int(by)
    msg_puck_center.bbox_w   = int(bw)
    msg_puck_center.bbox_h   = int(bh)
    msg_puck_center.color    = int(color_id)
    publisher_puck_center.publish(msg_puck_center)


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
    low, high = HSV_BOUNDS[color_name]
    mask = cv2.inRange(hsv_img, low, high)
    mask = preprocess_mask(mask)

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

    for color_name, color_id in [("red", 0), ("green", 1), ("blue", 2)]:
        detections, mask = detect_color_pucks(hsv_img, color_name, max_pucks=3)

        for contour, cx, cy in detections:
            # Bounding box in cropped-image space; offset y to full-image space
            bx, by_crop, bw, bh = cv2.boundingRect(contour)
            by = by_crop + crop_y
            publish_detection(header, cx, cy + crop_y, bx, by, bw, bh, color_id)

            if DEBUG_VISUALS:
                cv2.circle(img, (cx, cy), 2, (0, 255, 0), -1)
                cv2.rectangle(img, (bx, by_crop), (bx + bw, by_crop + bh), (255, 0, 0), 2)
                cv2.putText(img, color_name, (bx, max(0, by_crop - 6)),
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