#!/usr/bin/env python3
'''
CenterPuck service node.

Service:    /center_puck  (CenterPuckServerMessage)
  Request:  int32 puck_color   (0=red, 1=green, 2=blue)
  Response: bool  success

Subscribes: /camera/color/image_2fps/compressed
Publishes:  /cmd_vel (Twist)

On each camera frame: detects the requested puck, runs PID on angular.z
to drive it to the image centre-line, and publishes cmd_vel.
The service returns success once the puck stays aligned for
ALIGN_STABLE_FRAMES consecutive frames, or False on timeout.
'''

import math
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from foraging_msgs.srv import CenterPuckServerMessage, CenterPuckServerMessageResponse

# ── Tuning ───────────────────────────────────────────────────────────────────
DEBUG               = True
CROP_TOP_FRACTION   = 0.5

MIN_CONTOUR_AREA    = 100
MAX_CONTOUR_AREA    = 10000

Kp = 0.004
Ki = 0.00005
Kd = 0.002
MAX_ANGULAR_Z       = 0.8

ALIGN_TOLERANCE_PX  = 20    # within this many px = aligned
ALIGN_STABLE_FRAMES = 5     # consecutive aligned frames to declare success
SERVICE_TIMEOUT_S   = 25.0

SEARCH_ANGULAR_SPEED         = 0.5           # rad/s for the search sweep
SEARCH_ANGLE_RAD             = 0.5 # around 30 degrees
NO_PUCK_FRAMES_BEFORE_SEARCH = 10            # consecutive no-detection frames before triggering sweep

COLOR_NAMES = {0: "red", 1: "green", 2: "blue"}
HSV_BOUNDS = {
    "red":   [(np.array([  0, 105,   0]), np.array([ 15, 255, 255])),
              (np.array([165, 105,   0]), np.array([180, 255, 255]))],
    "green": [(np.array([ 35,  20,   0]), np.array([ 80, 255, 255]))],
    "blue":  [(np.array([ 90, 160,   0]), np.array([130, 255, 255]))],
}
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
# ─────────────────────────────────────────────────────────────────────────────

bridge      = CvBridge()
cmd_vel_pub = None

# PID state
error_sum  = 0.0
last_error = 0.0


def reset_pid():
    global error_sum, last_error
    error_sum  = 0.0
    last_error = 0.0


def pid(error):
    global error_sum, last_error
    error_sum += error
    diff        = error - last_error
    last_error  = error
    raw = Kp * error + Ki * error_sum + Kd * diff
    return max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, raw))


def preprocess_mask(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    _, mask = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
    return mask


def find_puck_cx(hsv_crop, color_name):
    """Return x-centre of the largest valid puck contour, or None."""
    ranges = HSV_BOUNDS[color_name]
    mask = preprocess_mask(
        np.bitwise_or.reduce([cv2.inRange(hsv_crop, lo, hi) for lo, hi in ranges])
    )
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        area = cv2.contourArea(c)
        perim = cv2.arcLength(c, True)
        circ = (4.0 * np.pi * area / (perim * perim)) if perim > 0 else 0
        if not (MIN_CONTOUR_AREA <= area <= MAX_CONTOUR_AREA):
            continue
        if perim <= 0:
            continue
        if circ < 0.4:
            continue
        if best is None or area > best[0]:
            best = (area, c)

    if best is None:
        return None

    M = cv2.moments(best[1])
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"])


def _rotate_step(angular_z):
    """Rotate by SEARCH_ANGLE_RAD at SEARCH_ANGULAR_SPEED, then stop."""
    duration = SEARCH_ANGLE_RAD / SEARCH_ANGULAR_SPEED
    t = Twist()
    t.angular.z = angular_z
    end = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        cmd_vel_pub.publish(t)
        cv2.waitKey(50)  # 50 ms — keeps the OpenCV window responsive during rotation
    cmd_vel_pub.publish(Twist())


def _puck_visible(color_name):
    """Grab one camera frame, optionally show it in the debug window, and
    return True if the puck is detected."""
    try:
        msg = rospy.wait_for_message(
            "/camera/color/image_2fps/compressed",
            CompressedImage,
            timeout=1.0,
        )
        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, _ = img.shape[:2]
        crop_y = int(h * CROP_TOP_FRACTION)
        crop = img[crop_y:, :]
        hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        cx = find_puck_cx(hsv_crop, color_name)

        if DEBUG:
            dbg = crop.copy()
            cv2.putText(dbg, "searching...", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if cx is not None:
                cv2.circle(dbg, (cx, dbg.shape[0] // 2), 6, (0, 255, 0), -1)
            cv2.imshow("center_puck", dbg)
            cv2.waitKey(1)

        return cx is not None
    except rospy.ROSException:
        return False


def search_sweep(color_name):
    """
    Called when the puck is not in the robot's FOV.
    Rotates in discrete steps (rotate → stop → check), covering 60° total,
    then returns to the original heading.

    Sweep pattern:
        0°  → –30° (1 step CW)   → check
        –30° → 0°  (1 step CCW)  → check
        0°  → +30° (1 step CCW)  → check
        +30° → 0°  (1 step CW)   → back to start (no check needed)

    Returns True as soon as the puck is spotted, False if never found.
    """
    rospy.loginfo("center_puck: puck not in FOV — starting search sweep")

    # Step 1: rotate 30° CW, then check
    _rotate_step(-SEARCH_ANGULAR_SPEED)
    if _puck_visible(color_name):
        return True

    # Step 2: rotate 30° CCW back to start, then check
    _rotate_step(SEARCH_ANGULAR_SPEED)
    if _puck_visible(color_name):
        return True

    # Step 3: rotate 30° CCW past start, then check
    _rotate_step(SEARCH_ANGULAR_SPEED)
    if _puck_visible(color_name):
        return True

    # Not found — rotate 30° CW back to original heading
    rospy.loginfo("center_puck: puck not found — returning to original heading")
    _rotate_step(-SEARCH_ANGULAR_SPEED)
    return False


def handle_center_puck(req):
    color_id = req.puck_color
    if color_id not in COLOR_NAMES:
        rospy.logwarn("center_puck: unknown color id %d", color_id)
        return CenterPuckServerMessageResponse(success=False)

    color_name        = COLOR_NAMES[color_id]
    rate              = rospy.Rate(10)
    deadline          = rospy.Time.now() + rospy.Duration(SERVICE_TIMEOUT_S)
    stable_count      = 0
    no_puck_count     = 0
    reset_pid()

    rospy.loginfo("center_puck: aligning to %s puck", color_name)

    try:
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logwarn("center_puck: timed out")
                cmd_vel_pub.publish(Twist())
                return CenterPuckServerMessageResponse(success=False)

            msg = rospy.wait_for_message(
                "/camera/color/image_2fps/compressed",
                CompressedImage,
                timeout=2.0
            )

            img            = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            full_h, full_w = img.shape[:2]
            crop_y         = int(full_h * CROP_TOP_FRACTION)
            crop           = img[crop_y:, :]
            hsv_crop       = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            desired_x      = full_w // 2

            cx = find_puck_cx(hsv_crop, color_name)

            twist = Twist()
            if cx is not None:
                no_puck_count   = 0
                error           = desired_x - cx
                twist.angular.z = pid(error)

                if abs(error) < ALIGN_TOLERANCE_PX:
                    stable_count += 1
                else:
                    stable_count = 0

                if stable_count >= ALIGN_STABLE_FRAMES:
                    cmd_vel_pub.publish(Twist())
                    rospy.loginfo("center_puck: aligned (error=%.1fpx)", error)
                    return CenterPuckServerMessageResponse(success=True)
            else:
                stable_count  = 0
                no_puck_count += 1
                if no_puck_count >= NO_PUCK_FRAMES_BEFORE_SEARCH:
                    no_puck_count = 0
                    reset_pid()
                    if not search_sweep(color_name):
                        rospy.logwarn("center_puck: puck not found after sweep")
                        cmd_vel_pub.publish(Twist())
                        return CenterPuckServerMessageResponse(success=False)
                    continue  # re-enter loop to grab a fresh frame after the sweep

            cmd_vel_pub.publish(twist)

            if DEBUG:
                dbg = crop.copy()
                cv2.line(dbg, (desired_x, 0), (desired_x, dbg.shape[0]), (0, 0, 255), 2)
                if cx is not None:
                    cv2.circle(dbg, (cx, dbg.shape[0] // 2), 6, (0, 255, 0), -1)
                    cv2.putText(dbg, f"err:{desired_x - cx:+d}px", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    cv2.putText(dbg, "no puck", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("center_puck", dbg)
                cv2.waitKey(1)

            rate.sleep()

        return CenterPuckServerMessageResponse(success=False)
    finally:
        cv2.destroyWindow("center_puck")


def main():
    global cmd_vel_pub
    rospy.init_node("center_puck_node")
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Service("/center_puck", CenterPuckServerMessage, handle_center_puck)
    rospy.loginfo("center_puck_node ready — service at /center_puck")
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
