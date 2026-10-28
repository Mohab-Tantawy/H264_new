import os
import time
import tempfile
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2


def find_working_h264_writer(w, h, fps):
    """
    Probe once at startup for a VideoWriter config that actually opens
    with H.264 on this machine. Returns (backend, fourcc) to reuse for
    every frame, instead of re-probing each time (expensive).
    """
    candidates = [
        (cv2.CAP_MSMF, cv2.VideoWriter_fourcc(*'H264')),
        (cv2.CAP_MSMF, cv2.VideoWriter_fourcc(*'avc1')),
        (cv2.CAP_ANY,  cv2.VideoWriter_fourcc(*'H264')),
        (cv2.CAP_ANY,  cv2.VideoWriter_fourcc(*'avc1')),
    ]
    tmp_path = os.path.join(tempfile.gettempdir(), '_h264_probe.mp4')
    for backend, fourcc in candidates:
        writer = cv2.VideoWriter(tmp_path, backend, fourcc, fps, (w, h))
        ok = writer.isOpened()
        writer.release()
        if ok:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            return backend, fourcc
    if os.path.exists(tmp_path):
        os.remove(tmp_path)
    return None, None


class UsbCameraPublisherH264(Node):
    """
    Same role as the original JPEG publisher, but each frame is encoded
    as an independent single-frame H.264 file (via cv2.VideoWriter) and
    published as CompressedImage bytes with format='h264'. This is
    intra-frame-only H.264 -- it does NOT get H.264's inter-frame/
    temporal compression benefit, since there's no persistent encoder
    state across frames in this design. It DOES give you H.264's
    per-frame compression efficiency vs. JPEG, and real timing numbers
    for comparison.
    """

    def __init__(self):
        super().__init__('usb_camera_publisher_h264')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('topic', '/camera/image_raw')

        device_index = self.get_parameter('device_index').value
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(CompressedImage, topic, qos_profile)

        self.cap = cv2.VideoCapture(device_index if device_index is not None else 0)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {device_index}')
            raise RuntimeError('Camera not available')

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError('Failed to grab a test frame to determine size')
        self.h, self.w = frame.shape[:2]

        self.backend, self.fourcc = find_working_h264_writer(self.w, self.h, rate)
        if self.backend is None:
            self.get_logger().error(
                'No working H.264 VideoWriter backend found on this machine. '
                'Falling back to mp4v (NOT real H.264) so the pipeline still runs.'
            )
            self.backend = cv2.CAP_ANY
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        self._tmp_path = os.path.join(tempfile.gettempdir(), '_h264_frame.mp4')

        self.get_logger().info(
            f'Camera opened at /dev/video{device_index} ({self.w}x{self.h}), '
            f'publishing to {topic} via per-frame H.264 (backend={self.backend})'
        )

        self._frame_count = 0
        self._encode_time_sum_ms = 0.0
        self._size_sum_bytes = 0

        self.create_timer(1.0 / rate, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        stamp = self.get_clock().now()

        t0 = time.perf_counter()
        writer = cv2.VideoWriter(self._tmp_path, self.backend, self.fourcc, 15.0, (self.w, self.h))
        writer.write(frame)
        writer.release()

        try:
            with open(self._tmp_path, 'rb') as f:
                data = f.read()
        except FileNotFoundError:
            self.get_logger().warn('Encoded frame file missing, skipping frame')
            return
        finally:
            if os.path.exists(self._tmp_path):
                os.remove(self._tmp_path)
        t1 = time.perf_counter()

        msg = CompressedImage()
        msg.format = "h264"
        msg.data = data
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        encode_ms = (t1 - t0) * 1000.0
        self._frame_count += 1
        self._encode_time_sum_ms += encode_ms
        self._size_sum_bytes += len(data)
        avg_ms = self._encode_time_sum_ms / self._frame_count
        avg_kb = (self._size_sum_bytes / self._frame_count) / 1024.0

        self.get_logger().info(
            f'[h264] encode+publish: {encode_ms:.2f} ms, size: {len(data)/1024:.1f} KB '
            f'(avg: {avg_ms:.2f} ms, avg size: {avg_kb:.1f} KB, frame #{self._frame_count})'
        )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraPublisherH264()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
