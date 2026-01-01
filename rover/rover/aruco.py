#!/usr/bin/env python3
"""
aruco_center_controller.py

ROS2 node that:
 - grabs camera frames using OpenCV (cv2.VideoCapture),
 - detects ArUco markers,
 - finds the largest detected marker (closest),
 - computes horizontal offset from frame center and distance using the provided polynomial,
 - applies a proportional controller:
     angular.z <- steer to reduce horizontal offset (if marker is right, give left angular velocity)
     linear.x  <- move forward/back to achieve a desired distance
 - publishes geometry_msgs/Twist on /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import time

# ---------- Configuration ----------
CAMERA_ID = 2              # OpenCV camera index
MARKER_REAL_SIZE = 295.0   # mm - set to the correct physical marker size (user noted 295mm)
ARUCO_DICT = cv2.aruco.DICT_6X6_250

# Controller gains and limits
KP_ANG = 0.8               # proportional gain for angular velocity
KP_LIN = 0.02              # proportional gain for linear velocity (per cm distance error)
MAX_ANG_VEL = 0.8          # rad/s
MAX_LIN_VEL = 0.35         # m/s

# Distance setpoint
TARGET_DISTANCE_CM = 80.0  # desired distance from marker in cm
DEADZONE_PIXELS = 10       # pixels tolerance for center (no angular correction if inside)

# Publish rate
PUBLISH_RATE_HZ = 10.0


# ---------- Calibrated Polynomial Model ----------
# y = -3.366e-06 * x^2 + 0.08133 * x + 106.3
# input: x (computed inverse-area metric), returns estimated distance in cm
def calibrated_model(x):
    return -3.366e-06 * (x ** 2) + 0.08133 * x + 106.3


# ---------- Utility: detect markers and choose best ----------
def detect_markers_and_largest(frame, detector, marker_size_mm=MARKER_REAL_SIZE):
    """
    Detect ArUco markers. Return (best_detection, annotated_frame)
    best_detection is a dict: {'id': int, 'area': float, 'center': (cx,cy), 'x_metric': float}
    or None if no marker found.
    """
    corners, ids, _ = detector.detectMarkers(frame)
    best = None

    if ids is None:
        return None, frame

    for i in range(len(ids)):
        pts = corners[i][0]            # shape (4,2)
        area_pixels = cv2.contourArea(np.int32(pts))
        if area_pixels <= 0:
            continue

        # inverse-area-based x metric (same as your original code)
        x_computed = (marker_size_mm ** 2) / area_pixels * 100.0

        cX = float(np.mean(pts[:, 0]))
        cY = float(np.mean(pts[:, 1]))

        # choose marker with largest area (closest)
        if (best is None) or (area_pixels > best['area']):
            best = {
                'id': int(ids[i][0]),
                'area': float(area_pixels),
                'center': (cX, cY),
                'x_metric': float(x_computed),
                'corners': pts
            }

    # annotate all detected markers (small green boxes)
    for i in range(len(ids)):
        pts = np.int32(corners[i][0])
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        # label each with its ID (small)
        cx, cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
        cv2.putText(frame, f"ID:{int(ids[i][0])}", (int(cx) - 25, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # highlight best marker in yellow+show area
    if best is not None:
        cv2.polylines(frame, [np.int32(best['corners'])], True, (0, 255, 255), 3)
        cv2.putText(frame, f"Best ID:{best['id']} Area:{best['area']:.0f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    return best, frame


# ---------- ROS2 Node ----------
class ArucoCenterController(Node):
    def __init__(self):
        super().__init__('aruco_center_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # OpenCV video capture
        self.cap = cv2.VideoCapture(CAMERA_ID)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open camera id {CAMERA_ID}")
            raise RuntimeError(f"Unable to open camera id {CAMERA_ID}")

        # ArUco detector (new API)
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # timer for periodic processing
        period = 1.0 / PUBLISH_RATE_HZ
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info("Aruco center controller started")

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Frame read failed; skipping this cycle")
            return

        height, width = frame.shape[:2]
        frame_center_x = width / 2.0

        detection, annotated = detect_markers_and_largest(frame, self.detector, MARKER_REAL_SIZE)

        twist = Twist()
        if detection is not None:
            # horizontal offset in pixels: positive => marker is to the right of center
            marker_cx = detection['center'][0]
            error_pixels = marker_cx - frame_center_x

            # angular control: if marker is right (error positive), want negative angular.z (turn left)
            # normalized error in [-1, 1]
            norm_err = error_pixels / (frame_center_x)
            # deadzone to reduce oscillation
            if abs(error_pixels) < DEADZONE_PIXELS:
                ang_correction = 0.0
            else:
                ang_correction = -KP_ANG * norm_err
                # clamp
                ang_correction = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, ang_correction))

            # distance estimate using calibrated polynomial (in cm)
            x_metric = detection['x_metric']
            est_distance_cm = calibrated_model(x_metric)

            # linear control to reach TARGET_DISTANCE_CM (positive error -> we are too far -> move forward)
            distance_error = est_distance_cm - TARGET_DISTANCE_CM
            lin_corr = KP_LIN * distance_error  # units: m/s if KP_LIN tuned accordingly (we assume cm->m factor in KP)
            # The calibrated_model returns cm; convert error to meters inside control or tune KP appropriately.
            # Here KP_LIN expects m/s per cm; clamp final linear velocity
            lin_corr = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, lin_corr))

            # If the marker is not roughly centered angularly, optionally reduce forward speed to avoid moving while not facing marker.
            # Simple strategy: scale linear by how centered we are
            centering_scale = max(0.0, 1.0 - min(1.0, abs(norm_err)))
            twist.linear.x = lin_corr * centering_scale
            twist.angular.z = ang_correction

            # Draw some overlays
            cv2.line(annotated, (int(frame_center_x), 0), (int(frame_center_x), height), (255, 255, 0), 1)
            cv2.circle(annotated, (int(marker_cx), int(detection['center'][1])), 6, (0, 0, 255), -1)
            cv2.putText(annotated, f"EstDist: {est_distance_cm:.1f} cm", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, f"ErrPx: {error_pixels:.1f} Norm: {norm_err:.3f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(annotated, f"cmd_lin: {twist.linear.x:.3f} m/s cmd_ang: {twist.angular.z:.3f} rad/s",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            self.get_logger().debug(
                f"Detected ID {detection['id']} area {detection['area']:.1f} err_px {error_pixels:.1f} est_dist {est_distance_cm:.2f}cm -> lin {twist.linear.x:.3f}, ang {twist.angular.z:.3f}"
            )

        else:
            # No marker found: publish zero velocities (or optionally rotate slowly to search)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cv2.putText(annotated, "No marker detected", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Publish twist
        self.pub.publish(twist)

        # Show image for debugging
        cv2.imshow("Aruco Center Controller", annotated)
        # use small waitKey here to keep imshow responsive
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # user-requested quit: gracefully shutdown the node
            self.get_logger().info("User requested shutdown via 'q' key")
            rclpy.shutdown()

    def destroy_node_and_capture(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        try:
            if self.cap.isOpened():
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


# ---------- Main ----------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoCenterController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Exception:", str(e))
    finally:
        if node is not None:
            node.destroy_node_and_capture()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
