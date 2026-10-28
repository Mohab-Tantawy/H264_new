import os
import tempfile
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2


class ViewerH264(Node):
    """
    Subscribes to CompressedImage messages containing per-frame H.264
    data (format='h264'), decodes each one by writing it to a temp file
    and reading it back with cv2.VideoCapture (OpenCV has no direct
    "decode H.264 bytes from memory" API), and displays it.
    """

    def __init__(self):
        super().__init__('viewer_node_h264')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage, '/camera/image_raw', self.camera, qos_profile)

        self._tmp_path = os.path.join(tempfile.gettempdir(), '_h264_frame_rx.mp4')

        self._frame_count = 0
        self._latency_sum_ms = 0.0
        self._last_arrival = None

        self.get_logger().info('Viewer initialized (per-frame H.264 decode)')

    def camera(self, msg):
        try:
            now = self.get_clock().now()
            stamp = Time.from_msg(msg.header.stamp)
            latency_ms = (now - stamp).nanoseconds / 1e6

            with open(self._tmp_path, 'wb') as f:
                f.write(msg.data)

            cap = cv2.VideoCapture(self._tmp_path)
            frame = None
            if cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    frame = None
            cap.release()

            if os.path.exists(self._tmp_path):
                os.remove(self._tmp_path)

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
                f'[h264] end-to-end latency: {latency_ms:.2f} ms '
                f'(avg: {avg_latency_ms:.2f} ms{inst_fps_str}, frame #{self._frame_count})'
            )

            if frame is not None:
                cv2.imshow('Camera Feed (H.264)', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Viewer error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ViewerH264()
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
