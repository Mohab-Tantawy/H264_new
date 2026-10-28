// camera_publisher_ffmpeg.cpp
//
// Publishes a USB camera feed over ROS2 using image_transport's "ffmpeg"
// plugin (ffmpeg_image_transport), which encodes frames to H.264 with a
// persistent libx264 encoder -- real inter-frame compression, not a
// per-frame reinitialized encoder.
//
// Requires: ros-jazzy-ffmpeg-image-transport (Linux only -- no Windows
// build exists on the RoboStack/pixi channel as of this writing).

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraPublisherFfmpeg : public rclcpp::Node
{
public:
    CameraPublisherFfmpeg() : Node("camera_publisher_ffmpeg"), frame_count_(0), encode_time_sum_ms_(0.0)
    {
        this->declare_parameter("device_index", 0);
        this->declare_parameter("publish_rate", 15.0);
        this->declare_parameter("frame_id", std::string("camera"));
        this->declare_parameter("topic", std::string("/camera/image_raw"));
        this->declare_parameter("capture_width", 640);
        this->declare_parameter("capture_height", 480);

        int device_index = this->get_parameter("device_index").as_int();
        double rate = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string topic = this->get_parameter("topic").as_string();
        int cap_width = this->get_parameter("capture_width").as_int();
        int cap_height = this->get_parameter("capture_height").as_int();

        cap_.open(device_index);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera at index %d", device_index);
            throw std::runtime_error("Camera not available");
        }

        // 16-aligned resolution avoids H.264 macroblock padding edge cases.
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, cap_width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cap_height);
        // Always grab the newest frame rather than a stale buffered one.
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        double actual_w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(this->get_logger(), "Requested %dx%d, camera gave %gx%g",
                    cap_width, cap_height, actual_w, actual_h);

        // Sensible real-time-friendly defaults for the ffmpeg transport,
        // overridable at launch with --ros-args -p camera.image_raw.ffmpeg.<name>:=<value>
        this->declare_parameter("camera.image_raw.ffmpeg.bit_rate", 3000000);
        this->declare_parameter("camera.image_raw.ffmpeg.gop_size", 15);
        this->declare_parameter("camera.image_raw.ffmpeg.max_b_frames", 0);
        this->declare_parameter("camera.image_raw.ffmpeg.pixel_format", std::string("yuv420p"));
        this->declare_parameter("camera.image_raw.ffmpeg.encoder_av_options",
                                 std::string("preset:fast,tune:zerolatency"));
        this->declare_parameter("image_transport", "ffmpeg");

        // RELIABLE QoS: H.264 messages are large (tens of KB) compared to
        // JPEG; BEST_EFFORT risks partial message loss under load, which
        // manifests as corrupted/incomplete decoded frames.
        rclcpp::QoS qos(10);
        qos.reliable();
        pub_ = image_transport::create_publisher(this, topic, qos.get_rmw_qos_profile());

        RCLCPP_INFO(this->get_logger(), "Publishing to %s via ffmpeg transport", topic.c_str());

        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&CameraPublisherFfmpeg::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            return;
        }

        auto stamp = this->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id_;
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        auto t0 = std::chrono::steady_clock::now();
        pub_.publish(*msg);
        auto t1 = std::chrono::steady_clock::now();

        double encode_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        frame_count_++;
        encode_time_sum_ms_ += encode_ms;
        double avg_ms = encode_time_sum_ms_ / frame_count_;

        RCLCPP_INFO(this->get_logger(), "[ffmpeg] encode+publish: %.2f ms (avg: %.2f ms, frame #%d)",
                    encode_ms, avg_ms, frame_count_);
    }

    cv::VideoCapture cap_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
    int frame_count_;
    double encode_time_sum_ms_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CameraPublisherFfmpeg>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_publisher_ffmpeg"), "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
