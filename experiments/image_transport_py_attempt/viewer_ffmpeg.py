import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge
from image_transport_py import ImageTransport
import cv2


class ViewerFfmpeg(Node):
    """
    Same role as the original Viewer node, but subscribes through
    image_transport using the "ffmpeg" transport instead of subscribing
    directly to a CompressedImage topic. The plugin decodes H.264 packets
    back into a raw sensor_msgs/Image before this callback ever sees them.
    """

    def __init__(self):
        super().__init__('viewer_node_ffmpeg')
        self.bridge = CvBridge()

        self.image_transport = ImageTransport(self, image_transport='ffmpeg')
        self.sub = self.image_transport.subscribe('/camera/image_raw', 1, self.camera_cb)

        # Rolling timing stats
        self._frame_count = 0
        self._latency_sum_ms = 0.0
        self._last_arrival = None

        self.get_logger().info('Viewer initialized (ffmpeg/H.264 transport)')

    def camera_cb(self, msg):
        try:
            now = self.get_clock().now()

            # End-to-end latency: capture timestamp (set on the publisher
            # right before encode+publish) vs. now (after network transit
            # + H.264 decode). This is directly comparable to the same
            # metric computed in the JPEG viewer.
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
                f'[ffmpeg] end-to-end latency: {latency_ms:.2f} ms '
                f'(avg: {avg_latency_ms:.2f} ms{inst_fps_str}, frame #{self._frame_count})'
            )

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is not None:
                cv2.imshow('Camera Feed (ffmpeg/H.264)', frame)
            cv2.waitKey(1)  # for testing, not efficient
        except Exception as e:
            self.get_logger().error(f'Viewer error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ViewerFfmpeg()
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
