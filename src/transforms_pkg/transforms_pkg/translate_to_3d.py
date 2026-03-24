#!/usr/bin/env python3
"""
translate_to_3d.py
------------------
ROS2 Jazzy node that lifts 2D YOLO detections into 3D by sampling the
aligned depth frame across the full bounding box region.

Pipeline
  /yolo/detections          ──┐
                               ├─► ApproximateTimeSynchronizer ──► /yolo/detections_3d_fast
  /camera/.../aligned_depth ──┘

Depth is sampled across the full (shrunk) bounding box region to maximise
valid pixel count and improve median robustness over objects of any size.

Subscribes
  /yolo/detections                                    yolo_msgs/DetectionArray
  /camera/camera/aligned_depth_to_color/image_raw     sensor_msgs/Image  (16UC1, mm)
  /camera/camera/aligned_depth_to_color/camera_info   sensor_msgs/CameraInfo

Publishes
  /yolo/detections_3d_fast                            yolo_msgs/DetectionArray

Parameters
  bbox_shrink  (float, default 0.8)   scale factor applied to bbox before sampling
                                      (avoids mixed-depth pixels at object edges)
  min_depth_mm (float, default 150)   ignore depth readings below this (noise floor)
  max_depth_mm (float, default 8000)  ignore depth readings above this
  slop         (float, default 0.05)  ApproximateTimeSynchronizer tolerance (s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from yolo_msgs.msg import DetectionArray


# ── QoS matching RealSense driver (BEST_EFFORT) ───────────────────────────────
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TranslateTo3DNode(Node):

    def __init__(self):
        super().__init__('translate_to_3d')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('bbox_shrink',  0.8)
        self.declare_parameter('min_depth_mm', 150.0)
        self.declare_parameter('max_depth_mm', 8000.0)
        self.declare_parameter('slop',         0.05)

        self._shrink  = self.get_parameter('bbox_shrink').value
        self._min_d   = self.get_parameter('min_depth_mm').value
        self._max_d   = self.get_parameter('max_depth_mm').value
        slop          = self.get_parameter('slop').value

        # ── State ────────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None

        # ── CameraInfo — grab once then unsubscribe ───────────────────────────
        self._info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self._camera_info_cb,
            10,
        )

        # ── Synchronised subscribers ──────────────────────────────────────────
        self._det_sub = message_filters.Subscriber(
            self, DetectionArray,
            '/yolo/detections',
            qos_profile=10,
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=_SENSOR_QOS,
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._det_sub, self._depth_sub],
            queue_size=10,
            slop=slop,
        )
        self._sync.registerCallback(self._synced_cb)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(DetectionArray, '/yolo/detections_3d_fast', 10)

        self.get_logger().info(
            f'translate_to_3d ready  '
            f'[bbox_shrink={self._shrink}  '
            f'depth={self._min_d:.0f}–{self._max_d:.0f} mm  '
            f'slop={slop}s]'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._fx is not None:
            return
        k = msg.k
        self._fx, self._fy = k[0], k[4]
        self._cx, self._cy = k[2], k[5]
        self.get_logger().info(
            f'Intrinsics cached  fx={self._fx:.2f}  fy={self._fy:.2f}  '
            f'cx={self._cx:.2f}  cy={self._cy:.2f}'
        )
        self.destroy_subscription(self._info_sub)

    def _synced_cb(self, det_msg: DetectionArray, depth_msg: Image) -> None:
        if self._fx is None:
            self.get_logger().warn('Intrinsics not yet received — dropping frame',
                                   throttle_duration_sec=2.0)
            return

        # 16UC1 numpy array — pixel values already in mm
        depth_arr: np.ndarray = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )
        img_h, img_w = depth_arr.shape

        out = DetectionArray()
        out.header = det_msg.header

        for det in det_msg.detections:
            cx_px = det.bbox.center.position.x  # pixel centre X
            cy_px = det.bbox.center.position.y  # pixel centre Y
            bw    = det.bbox.size.x              # bbox width  (pixels)
            bh    = det.bbox.size.y              # bbox height (pixels)

            depth_mm = self._sample_bbox_depth(depth_arr, cx_px, cy_px, bw, bh, img_h, img_w)

            if depth_mm > 0.0:
                depth_m = depth_mm / 1000.0
                det.bbox3d.center.position.x = (cx_px - self._cx) * depth_m / self._fx
                det.bbox3d.center.position.y = (cy_px - self._cy) * depth_m / self._fy
                det.bbox3d.center.position.z = depth_m
                det.bbox3d.center.orientation.w = 1.0
                det.bbox3d.frame_id = det_msg.header.frame_id
            else:
                self.get_logger().debug(
                    f'No valid depth for {det.class_name} '
                    f'bbox=({cx_px:.0f},{cy_px:.0f}) '
                    f'size=({bw:.0f}x{bh:.0f})'
                )

            out.detections.append(det)

        self._pub.publish(out)

    # ── Depth sampling ────────────────────────────────────────────────────────

    def _sample_bbox_depth(
        self,
        depth_arr: np.ndarray,
        cx: float, cy: float,
        bw: float, bh: float,
        img_h: int, img_w: int,
    ) -> float:
        """
        Sample depth across the (shrunk) bounding box region.

        The bbox is shrunk by `bbox_shrink` before sampling to avoid
        mixed-depth edge pixels where the object meets the background.

        Returns the median depth in mm, or 0.0 if no valid pixels found.
        """
        half_w = (bw * self._shrink) / 2.0
        half_h = (bh * self._shrink) / 2.0

        u0 = max(0,     int(cx - half_w))
        u1 = min(img_w, int(cx + half_w))
        v0 = max(0,     int(cy - half_h))
        v1 = min(img_h, int(cy + half_h))

        if u1 <= u0 or v1 <= v0:
            # Bbox collapsed after shrinking (tiny detection) — fall back to single pixel
            u = int(np.clip(round(cx), 0, img_w - 1))
            v = int(np.clip(round(cy), 0, img_h - 1))
            d = float(depth_arr[v, u])
            return d if self._min_d <= d <= self._max_d else 0.0

        region = depth_arr[v0:v1, u0:u1].astype(np.float32)
        valid  = region[(region >= self._min_d) & (region <= self._max_d)]

        return float(np.median(valid)) if valid.size > 0 else 0.0


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TranslateTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
