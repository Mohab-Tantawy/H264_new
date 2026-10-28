import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class Viewer(Node):
    def __init__(self):
        super().__init__('viewer_node')
        self.bridge = CvBridge()

        # Must match the QoS settings of the publishers to avoid dropped messages
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage, '/camera/image_raw', self.camera, qos_profile)

        # Rolling timing stats
        self._frame_count = 0
        self._latency_sum_ms = 0.0
        self._last_arrival = None

        self.get_logger().info('Viewer initialized with window (jpeg)')

    def camera(self, msg):
        try:
            now = self.get_clock().now()

            # Same latency definition as the ffmpeg viewer: capture
            # timestamp (set right before encode+publish) vs. now
            # (after network transit + JPEG decode).
            stamp = Time.from_msg(msg.header.stamp)
            latency_ms = (now - stamp).nanoseconds / 1e6

            inst_fps_str = ''
            if self._last_arrival is not None:
                dt_s = (now - self._last_arrival).nanoseconds / 1e9
                if dt_s > 0:
                    inst_fps_str = f', inst_fps: {1.0 / dt_s:.1f}'
            self._last_arrival = now

            self._frame_count += 1
            self._latency_sum_ms += latency_ms
            avg_latency_ms = self._latency_sum_ms / self._frame_count

            self.get_logger().info(
                f'[jpeg] end-to-end latency: {latency_ms:.2f} ms '
                f'(avg: {avg_latency_ms:.2f} ms{inst_fps_str}, frame #{self._frame_count})'
            )

            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if frame is not None:
                cv2.imshow('Camera Feed', frame)
            cv2.waitKey(1) # for testing, not effecient
        except Exception as e:
            self.get_logger().error(f'Camera 1 error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
