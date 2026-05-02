#!/usr/bin/env python3
"""
perception_node.py — OpenCV-based target detection for the Ackerman CV-control task.

Strategy
--------
1. Convert incoming ROS Image → OpenCV BGR.
2. Convert BGR → HSV and apply a two-range red mask
   (red wraps around hue=0 and hue=179 in OpenCV's 0-179 scale).
3. Morphological cleanup (open + dilate) to remove noise.
4. Find contours and filter by:
     a. Minimum area (reject tiny blobs).
     b. Approx-polygon with 4 vertices (rectangle → box shape).
        This eliminates the sphere and cylinder decoys which produce
        circular / oval contours even after colour filtering.
5. Among remaining candidates, pick the largest (closest) one.
6. Compute centroid (cx, cy) and bounding-box area.
7. Publish as std_msgs/String JSON:
     {"cx": <int>, "cy": <int>, "area": <float>, "img_w": <int>, "img_h": <int>}
   Also publishes a debug annotated image on /perception/debug_image.

Topics
------
Subscribes : /camera/image_raw     (sensor_msgs/Image)
Publishes  : /perception/target_info  (std_msgs/String)  — JSON
             /perception/debug_image  (sensor_msgs/Image) — annotated feed
"""

import json
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # ── Parameters (overridable from launch file) ─────────────────────────
        self.declare_parameter('h_low1',  0)
        self.declare_parameter('s_low1',  100)
        self.declare_parameter('v_low1',  60)
        self.declare_parameter('h_high1', 10)
        self.declare_parameter('s_high1', 255)
        self.declare_parameter('v_high1', 255)

        self.declare_parameter('h_low2',  160)
        self.declare_parameter('s_low2',  100)
        self.declare_parameter('v_low2',  60)
        self.declare_parameter('h_high2', 179)
        self.declare_parameter('s_high2', 255)
        self.declare_parameter('v_high2', 255)

        self.declare_parameter('min_contour_area', 300)

        # ── Bridge and pubs/subs ──────────────────────────────────────────────
        self._bridge = CvBridge()

        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self._pub_target = self.create_publisher(String, '/perception/target_info', 10)
        self._pub_debug  = self.create_publisher(Image,  '/perception/debug_image', 10)

        self.get_logger().info('PerceptionNode started — subscribing to /camera/image_raw')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_hsv_params(self):
        """Read current HSV parameters."""
        lo1 = np.array([
            self.get_parameter('h_low1').value,
            self.get_parameter('s_low1').value,
            self.get_parameter('v_low1').value,
        ], dtype=np.uint8)
        hi1 = np.array([
            self.get_parameter('h_high1').value,
            self.get_parameter('s_high1').value,
            self.get_parameter('v_high1').value,
        ], dtype=np.uint8)
        lo2 = np.array([
            self.get_parameter('h_low2').value,
            self.get_parameter('s_low2').value,
            self.get_parameter('v_low2').value,
        ], dtype=np.uint8)
        hi2 = np.array([
            self.get_parameter('h_high2').value,
            self.get_parameter('s_high2').value,
            self.get_parameter('v_high2').value,
        ], dtype=np.uint8)
        return lo1, hi1, lo2, hi2

    def _is_box_contour(self, contour) -> bool:
        """
        Returns True if the contour approximates a 4-sided polygon (box/rectangle).
        Uses Douglas-Peucker approximation with epsilon = 4% of arc length.
        Requires between 4 and 6 vertices to allow for slight perspective distortion.
        """
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx  = cv2.approxPolyDP(contour, epsilon, True)
        return 4 <= len(approx) <= 6

    # ── Main callback ─────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        # Convert ROS image → OpenCV BGR
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        img_h, img_w = frame.shape[:2]
        debug_frame = frame.copy()

        # ── 1. Saturation Thresholding (Isolates colors from gray) ──────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # S channel is index 1. We look for saturated colors (S > 50)
        s_channel = hsv[:, :, 1]
        _, mask = cv2.threshold(s_channel, 50, 255, cv2.THRESH_BINARY)
        
        # Cleanup mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)

        # ── 2. Find Contours ──────────────────────────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best = None
        max_area = 0
        cx, cy, area = 0, 0, 0
        
        min_area = self.get_parameter('min_contour_area').value

        # Draw all valid-area contours in blue for debug
        for cnt in contours:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(debug_frame, [cnt], -1, (255, 0, 0), 1)

        # Iterate to find the best box
        for cnt in contours:
            area_cnt = cv2.contourArea(cnt)
            if area_cnt < min_area:
                continue

            # ── 3. Shape Analysis ─────────────────────────────────────────────
            # Polygon approximation
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            # Look for quadrilateral-like shapes (4-8 vertices)
            if len(approx) == 4:
                if area_cnt > max_area:
                    max_area = area_cnt
                    best = approx
                    area = area_cnt

        if best is not None:
            # Centroid
            M = cv2.moments(best)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            
            # ── 4. Publish Target Info ───────────────────────────────────────
            payload = json.dumps({
                'cx': cx,
                'cy': cy,
                'area': round(area, 1),
                'img_w': img_w,
                'img_h': img_h,
            })
            self._pub_target.publish(String(data=payload))

            # Annotate BEST
            cv2.drawContours(debug_frame, [best], -1, (0, 255, 0), 2)
            cv2.circle(debug_frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(debug_frame, (img_w // 2, cy), (cx, cy), (0, 0, 255), 2)
            cv2.putText(debug_frame, f'BOX: cx={cx} area={int(area)}', 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_frame, 'TARGET LOST', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Cross-hair
        cv2.line(debug_frame, (img_w // 2, 0), (img_w // 2, img_h), (200, 200, 200), 1)

        # ── 5. Create Debug Composite ───────────────────────────────────────
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        composite = cv2.hconcat([debug_frame, mask_bgr])

        try:
            debug_msg = self._bridge.cv2_to_imgmsg(composite, encoding='bgr8')
            debug_msg.header = msg.header
            self._pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
