"""Vision bridge -- answers the orchestrator's "where is X?" question.

This node is the single seam between the perception pipeline and the motion
stack. It caches the 3D detections produced by ``localization_3d_node`` and,
on request, publishes the position of one requested object in the arm's
planning frame.

  REQUEST  in : /vision/find_request   std_msgs/String   ("cup" | "goal")
  RESPONSE out: /vision/object_point   geometry_msgs/PointStamped (planning frame)

That is the whole contract; see the VISION CONTRACT block in
``rx150_pick_place_orchestrator.py``. Whoever answers on the response topic
owns *how* things are found -- this node, or ``vision_placeholder``.

Two rules make the contract safe to drive a real arm with:

1. **Silence means "not found".** If nothing matches, this node publishes
   NOTHING. The orchestrator then times out and aborts cleanly. Publishing a
   zeroed PointStamped instead would hand the planner (0, 0, 0) -- the arm's own
   base -- as a grasp target.
2. **Detections expire.** A cached hit older than ``detection_ttl_sec`` is not an
   answer, it is a memory. Without this the arm would confidently reach for a cup
   that was carried off two minutes ago.

Requests are served asynchronously: if nothing fresh is cached when a request
lands, the node holds the request open for ``response_wait_sec`` and answers the
moment a matching detection arrives, rather than blocking the executor.
"""

import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String

from arm_interfaces.msg import DetectedObjectArray


class VisionBridgeNode(Node):
    def __init__(self):
        super().__init__('vision_bridge')

        self.declare_parameter('localization_3d_topic', '/perception/detections_3d')
        self.declare_parameter('object_request_topic', '/vision/find_request')
        self.declare_parameter('object_point_topic', '/vision/object_point')
        self.declare_parameter('clear_cache_topic', '/vision/clear_cache')
        self.declare_parameter('planning_frame', 'rx150/base_link')

        # Which detector class names satisfy each request kind. The orchestrator
        # asks for "cup" and "goal"; the detector speaks COCO//custom class names.
        # Keeping the mapping here means neither side has to know the other's
        # vocabulary, and adding a kind is a parameter change, not a code change.
        self.declare_parameter('cup_classes', ['cup'])
        self.declare_parameter('goal_classes', [''])

        # Static fallbacks, used ONLY when the matching classes yield nothing.
        self.declare_parameter('cup_fallback_xyz', [float('nan')])
        self.declare_parameter('goal_fallback_xyz', [0.20, 0.18, 0.16])
        # A cached detection older than this is stale and is not an answer.
        self.declare_parameter('detection_ttl_sec', 180.0)
        self.declare_parameter('response_wait_sec', 3.0)
        # Extra confidence gate on top of the detector's own threshold.
        self.declare_parameter('min_confidence', 0.0)
        # Added to the detected point before it is published. The detector
        # reports the bounding-box centre; nudge it here if the grasp point
        # should sit above or below that without touching the pipeline.
        self.declare_parameter('z_offset', 0.0)
        # Which detection to keep when several arrive for one class inside the
        # TTL window.
        #   'best'   -- keep the most confident (default)
        #   'latest' -- keep the newest
        self.declare_parameter('cache_policy', 'best')

        gp = self.get_parameter
        self._frame = str(gp('planning_frame').value)
        self._ttl_sec = float(gp('detection_ttl_sec').value)
        self._response_wait_sec = float(gp('response_wait_sec').value)
        self._min_confidence = float(gp('min_confidence').value)
        self._z_offset = float(gp('z_offset').value)
        self._cache_policy = str(gp('cache_policy').value).strip().lower()

        self._classes = {
            'cup': self._clean_classes(gp('cup_classes').value),
            'goal': self._clean_classes(gp('goal_classes').value),
        }
        self._fallbacks = {
            'cup': self._clean_xyz(gp('cup_fallback_xyz').value),
            'goal': self._clean_xyz(gp('goal_fallback_xyz').value),
        }

        self._cache = {}
        # An open request: (kind, monotonic deadline). At most one at a time.
        self._pending = None

        self._point_pub = self.create_publisher(
            PointStamped, str(gp('object_point_topic').value), 10
        )
        self.create_subscription(
            DetectedObjectArray, str(gp('localization_3d_topic').value),
            self._on_detections, 10,
        )
        self.create_subscription(
            String, str(gp('object_request_topic').value), self._on_request, 10
        )
        self.create_subscription(
            Empty, str(gp('clear_cache_topic').value), self._on_clear_cache, 10
        )
        self.create_timer(0.1, self._service_pending)

        self.get_logger().info(
            'vision_bridge up (frame %s, ttl %.0fs). cup<-%s  goal<-%s%s'
            % (
                self._frame, self._ttl_sec,
                self._classes['cup'] or 'nothing',
                self._classes['goal'] or 'nothing',
                ''.join(
                    '  [%s fallback %s]' % (kind, xyz)
                    for kind, xyz in self._fallbacks.items() if xyz
                ),
            )
        )

    # -- parameter helpers --------------------------------------------------
    @staticmethod
    def _clean_classes(value):
        """Drop the empty-string placeholder ROS needs for an empty str array."""
        return [str(v).strip() for v in (value or []) if str(v).strip()]

    @staticmethod
    def _clean_xyz(value):
        """Return [x, y, z] or None. NaN/short lists mean 'no fallback'."""
        try:
            xyz = [float(v) for v in (value or [])]
        except (TypeError, ValueError):
            return None
        if len(xyz) != 3 or any(v != v for v in xyz):  # v != v -> NaN
            return None
        return xyz

    # -- callbacks ----------------------------------------------------------
    def _on_detections(self, msg: DetectedObjectArray) -> None:
        now = time.monotonic()
        for det in msg.objects:
            if det.confidence < self._min_confidence:
                continue
            self._cache[det.class_name] = self._pick(
                self._cache.get(det.class_name), det, now)
        self._service_pending()

    def _pick(self, existing, det, now):
        """Choose between the cached detection for a class and a new one."""
        if existing is None or self._cache_policy == 'latest':
            return (det, now)
        prev_det, prev_seen = existing
        # A stale entry loses regardless of how confident it was.
        if now - prev_seen > self._ttl_sec:
            return (det, now)
        if det.confidence > prev_det.confidence:
            return (det, now)
        return existing

    def _on_clear_cache(self, _msg: Empty) -> None:
        """Drop every cached detection."""
        count = len(self._cache)
        self._cache.clear()
        self._pending = None
        self.get_logger().info(
            'Vision cache cleared (%d class(es) dropped); only detections from '
            'here on will answer.' % count
        )

    def _on_request(self, msg: String) -> None:
        kind = msg.data.strip().lower()
        if kind not in self._classes:
            self.get_logger().warning(
                "Unknown vision request '%s' (known: %s); ignoring."
                % (msg.data, ', '.join(sorted(self._classes)))
            )
            return
        self.get_logger().info("Vision request '%s'." % kind)
        if self._answer(kind):
            return
        # Nothing cached and fresh. Hold the request open rather than failing
        # now -- a detection may be one frame away.
        self._pending = (kind, time.monotonic() + self._response_wait_sec)

    def _service_pending(self) -> None:
        """Retry an open request; give up (or fall back) once it expires."""
        if self._pending is None:
            return
        kind, deadline = self._pending
        if self._answer(kind):
            self._pending = None
            return
        if time.monotonic() < deadline:
            return
        self._pending = None

        fallback = self._fallbacks.get(kind)
        if fallback is None:
            # Publish nothing. The orchestrator times out and aborts, which is
            # the correct outcome: we do not know where the object is.
            self.get_logger().error(
                "No fresh detection for '%s' (classes %s) within %.1f s and no "
                'fallback configured -- publishing nothing. The mission will '
                'abort rather than reach for a guessed position.'
                % (kind, self._classes[kind] or '[]', self._response_wait_sec)
            )
            return
        self.get_logger().warning(
            "No detection for '%s'; answering with the PLACEHOLDER fallback "
            '[%.3f, %.3f, %.3f]. This is a fixed point, not something the '
            'camera saw.' % (kind, *fallback)
        )
        self._publish(fallback, stamp=None)

    # -- answering ----------------------------------------------------------
    def _answer(self, kind: str) -> bool:
        """Publish the best fresh detection for ``kind``. True if one was sent.

        """
        now = time.monotonic()
        for class_name in self._classes[kind]:
            entry = self._cache.get(class_name)
            if entry is None:
                continue
            det, seen_at = entry
            if now - seen_at > self._ttl_sec:
                continue
            break
        else:
            return False

        point = [det.position_3d.x, det.position_3d.y, det.position_3d.z]
        frame = det.header.frame_id
        if frame and frame != self._frame:
            self.get_logger().warning(
                "Detection '%s' is in frame '%s', not the planning frame '%s'. "
                'localization_3d_node should transform to the planning frame; '
                'publishing the values as-is.' % (class_name, frame, self._frame)
            )

        self.get_logger().info(
            "Answered '%s' with %s (conf %.2f) -> [%.3f, %.3f, %.3f]"
            "  (detected z %.3f + z_offset %.3f)"
            % (kind, class_name, det.confidence,
               point[0], point[1], point[2] + self._z_offset,
               point[2], self._z_offset)
        )
        self._publish(point, stamp=det.header.stamp)
        return True

    def _publish(self, xyz, stamp) -> None:
        out = PointStamped()
        out.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.point.x = float(xyz[0])
        out.point.y = float(xyz[1])
        out.point.z = float(xyz[2]) + self._z_offset
        self._point_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
