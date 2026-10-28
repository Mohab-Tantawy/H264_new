import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from image_transport_py import ImageTransport
import cv2


class UsbCameraPublisherFfmpeg(Node):
    """
    Same as the original UsbCameraPublisher, but instead of manually
    JPEG-encoding each frame with cv2.imencode() and publishing a
    CompressedImage, this publishes a raw sensor_msgs/Image through
    image_transport's Publisher. The "ffmpeg" transport plugin
    (ffmpeg_image_transport) intercepts that publish() call and encodes
    the frame with H.264 before it goes on the wire. The subscriber side
    must use the matching image_transport Subscriber to decode it back.
    """

    def __init__(self):
        super().__init__('usb_camera_publisher_ffmpeg')

        # Parameters (same as original)
        self.declare_parameter('device_index', 0)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('topic', '/camera/image_raw')

        device_index = self.get_parameter('device_index').value
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        self.bridge = CvBridge()

        # image_transport=ffmpeg tells this ImageTransport instance to use
        # the ffmpeg_image_transport plugin by default for this publisher.
        # You can override this at runtime with the node parameter
        # "<node_name>.image_transport" instead of hardcoding it here, if
        # you want to switch transports without touching the code.
        self.image_transport = ImageTransport(self, image_transport='ffmpeg')
        self.pub = self.image_transport.advertise(topic, 1)

        self.cap = cv2.VideoCapture(device_index if device_index is not None else 0)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at index {device_index}')
            raise RuntimeError('Camera not available')

        self.get_logger().info(
            f'Camera opened at /dev/video{device_index}, publishing to {topic} via ffmpeg (H.264) transport'
        )

        # Rolling timing stats
        self._frame_count = 0
        self._encode_time_sum_ms = 0.0

        self.create_timer(1.0 / rate, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        # Timestamp at capture time -- this is what the receiver will use
        # to compute end-to-end latency, same convention as the JPEG version.
        stamp = self.get_clock().now()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id

        # This publish() call is where the H.264 encoding actually happens
        # (inside the ffmpeg_image_transport plugin), so timing around it
        # is the direct equivalent of timing cv2.imencode() in the JPEG version.
        t0 = time.perf_counter()
        self.pub.publish(msg)
        t1 = time.perf_counter()

        encode_publish_ms = (t1 - t0) * 1000.0
        self._frame_count += 1
        self._encode_time_sum_ms += encode_publish_ms
        avg_ms = self._encode_time_sum_ms / self._frame_count

        self.get_logger().info(
            f'[ffmpeg] encode+publish: {encode_publish_ms:.2f} ms '
            f'(avg: {avg_ms:.2f} ms, frame #{self._frame_count})'
        )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraPublisherFfmpeg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
