// viewer_ffmpeg.cpp
//
// Subscribes to a /camera/image_raw stream encoded via image_transport's
// "ffmpeg" plugin, decodes it (forcing software decode -- see decoders.h264
// param below, since VMs have no GPU passthrough for hardware decode),
// and displays it while logging end-to-end latency.

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

class ViewerFfmpeg : public rclcpp::Node
{
public:
    ViewerFfmpeg() : Node("viewer_node_ffmpeg"), frame_count_(0), latency_sum_ms_(0.0),
                      have_last_arrival_(false)
    {
        this->declare_parameter("topic", std::string("/camera/image_raw"));
        std::string topic = this->get_parameter("topic").as_string();
        this->declare_parameter("image_transport", "ffmpeg");

        // Force software H.264 decode -- the default tries hardware
        // (h264_cuvid / NVIDIA), which fails in a VM with no GPU passthrough.
        this->declare_parameter("camera.image_raw.ffmpeg.decoders.h264", std::string("h264"));

        // Must match the publisher's RELIABLE QoS or messages are dropped silently.
        rclcpp::QoS qos(10);
        qos.reliable();

        sub_ = image_transport::create_subscription(
            this, topic,
            std::bind(&ViewerFfmpeg::image_callback, this, std::placeholders::_1),
            "ffmpeg",
            qos.get_rmw_qos_profile());

        RCLCPP_INFO(this->get_logger(), "Viewer initialized (ffmpeg/H.264), subscribed to %s", topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        auto now = this->now();
        rclcpp::Time stamp(msg->header.stamp);
        double latency_ms = (now - stamp).nanoseconds() / 1e6;

        std::string inst_fps_str;
        if (have_last_arrival_) {
            double dt_s = (now - last_arrival_).nanoseconds() / 1e9;
            if (dt_s > 0) {
                inst_fps_str = ", inst_fps: " + std::to_string(1.0 / dt_s);
            }
        }
        last_arrival_ = now;
        have_last_arrival_ = true;

        frame_count_++;
        latency_sum_ms_ += latency_ms;
        double avg_latency_ms = latency_sum_ms_ / frame_count_;

        RCLCPP_INFO(this->get_logger(), "[ffmpeg] end-to-end latency: %.2f ms (avg: %.2f ms%s, frame #%d)",
                    latency_ms, avg_latency_ms, inst_fps_str.c_str(), frame_count_);

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::imshow("Camera Feed (ffmpeg/H.264)", cv_ptr->image);
            cv::waitKey(1);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    image_transport::Subscriber sub_;
    int frame_count_;
    double latency_sum_ms_;
    rclcpp::Time last_arrival_;
    bool have_last_arrival_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerFfmpeg>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
