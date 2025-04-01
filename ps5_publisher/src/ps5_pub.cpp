#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

class PS5PublisherNode : public rclcpp::Node {
public:
    PS5PublisherNode()
        : Node("ps5_publisher_node"), frame_count_(0), max_frames_(20), log_interval_(5),
          baseline_(0.1),  // Default baseline: 10 cm
          fov_(90.0) {     // Default FOV: 90 degrees
        publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("frame_left", 10);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("frame_right", 10);
        publisher_depth_ = this->create_publisher<sensor_msgs::msg::Image>("frame_depth", 10); // New publisher

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&PS5PublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "PS5PublisherNode initialized with 30 FPS target.");

        // Open video source
        video_.open(2);
        video_.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
        video_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
        video_.set(cv::CAP_PROP_FPS, 30);

        if (!video_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source.");
            rclcpp::shutdown();
        }
    }

private:
    void timerCallback() {
        cv::Mat frame;
        if (!video_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from video source.");
            return;
        }

        // Split stereo frames
        int mid_width = frame.cols / 2;
        cv::Mat left_frame = frame(cv::Range::all(), cv::Range(0, mid_width));
        cv::Mat right_frame = frame(cv::Range::all(), cv::Range(mid_width, frame.cols));

        // Compute depth map
        cv::Mat depth_map = computeDepthMap(left_frame, right_frame);

        // Publish messages
        publisher_left_->publish(convertCvToImgMsg(left_frame));
        publisher_right_->publish(convertCvToImgMsg(right_frame));
        publisher_depth_->publish(convertCvToDepthImgMsg(depth_map)); // Publish depth map

        // Log execution at intervals
        frame_count_++;
        if (frame_count_ % log_interval_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %d frames.", frame_count_);
        }
    }

    cv::Mat computeDepthMap(const cv::Mat &left_frame, const cv::Mat &right_frame) {
    cv::Mat gray_left, gray_right;
    cv::cvtColor(left_frame, gray_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_frame, gray_right, cv::COLOR_BGR2GRAY);

    cv::Mat disparity_map;

    int ndisparities = 16 * 5;   // Range of disparity
    int SADWindowSize = 21;     // Size of the block window. Must be odd

    // Use cv::StereoBM::create() to create the StereoBM object
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparities, SADWindowSize);

    // Compute the disparity map
    sbm->compute(gray_left, gray_right, disparity_map);

    // Convert disparity to depth using baseline and focal length
    cv::Mat depth_map = disparity_map;

    return depth_map;
}

    sensor_msgs::msg::Image convertCvToImgMsg(const cv::Mat &frame) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        return *msg;
    }

    sensor_msgs::msg::Image convertCvToDepthImgMsg(const cv::Mat &frame) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        return *msg;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_; // New publisher
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_;
    int frame_count_;
    const int max_frames_;
    const int log_interval_;
    double baseline_; // Distance between the lenses (in meters)
    double fov_;      // Field of view of the camera (in degrees)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PS5PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
