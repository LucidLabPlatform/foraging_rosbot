#!/usr/bin/env python3
"""
Interactive HSV range tuner with trackbars.
Shows the mask live so you can see exactly what gets detected.

Usage: python3 hsv_tuner.py --image my_photo.jpg
       python3 hsv_tuner.py                        # live ROS topic
"""

import cv2
import numpy as np
import argparse

WINDOW = "HSV Tuner"
MASK_WIN = "Mask Preview"

def nothing(x):
    pass

def make_trackbars(h_lo, h_hi, s_lo, s_hi, v_lo, v_hi):
    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("H low",  WINDOW, h_lo, 180, nothing)
    cv2.createTrackbar("H high", WINDOW, h_hi, 180, nothing)
    cv2.createTrackbar("S low",  WINDOW, s_lo, 255, nothing)
    cv2.createTrackbar("S high", WINDOW, s_hi, 255, nothing)
    cv2.createTrackbar("V low",  WINDOW, v_lo, 255, nothing)
    cv2.createTrackbar("V high", WINDOW, v_hi, 255, nothing)

def get_bounds():
    hl = cv2.getTrackbarPos("H low",  WINDOW)
    hh = cv2.getTrackbarPos("H high", WINDOW)
    sl = cv2.getTrackbarPos("S low",  WINDOW)
    sh = cv2.getTrackbarPos("S high", WINDOW)
    vl = cv2.getTrackbarPos("V low",  WINDOW)
    vh = cv2.getTrackbarPos("V high", WINDOW)
    return np.array([hl, sl, vl]), np.array([hh, sh, vh])

def process(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    # For red: you'd OR two ranges — add a second set of trackbars if needed
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    result = cv2.bitwise_and(img, img, mask=mask)
    return mask, result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, default=None)
    parser.add_argument("--topic", type=str, default="/camera/color/image_2fps/compressed")
    # Seed with rough values from Step 1 (edit these)
    parser.add_argument("--hlo", type=int, default=0)
    parser.add_argument("--hhi", type=int, default=180)
    parser.add_argument("--slo", type=int, default=50)
    parser.add_argument("--shi", type=int, default=255)
    parser.add_argument("--vlo", type=int, default=50)
    parser.add_argument("--vhi", type=int, default=255)
    args = parser.parse_args()

    make_trackbars(args.hlo, args.hhi, args.slo, args.shi, args.vlo, args.vhi)
    cv2.namedWindow(MASK_WIN)

    print("Drag sliders to refine. Press 'p' to print current bounds, 'q' to quit.")

    def run_loop(get_frame):
        while True:
            img = get_frame()
            if img is None:
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    break
                continue
            lower, upper = get_bounds()
            mask, result = process(img, lower, upper)
            cv2.imshow(WINDOW, result)
            cv2.imshow(MASK_WIN, mask)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            if key == ord('p'):
                print(f"\n>>> HSV_BOUNDS entry:")
                print(f'    "color": (np.array({list(lower)}), np.array({list(upper)})),')
        cv2.destroyAllWindows()

    if args.image:
        img = cv2.imread(args.image)
        run_loop(lambda: img)
    else:
        import rospy
        from sensor_msgs.msg import CompressedImage
        from cv_bridge import CvBridge
        bridge = CvBridge()
        latest = [None]

        def cb(msg):
            latest[0] = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        rospy.init_node("hsv_tuner", anonymous=True)
        rospy.Subscriber(args.topic, CompressedImage, cb, queue_size=1)
        run_loop(lambda: latest[0].copy() if latest[0] is not None else None)

if __name__ == "__main__":
    main()