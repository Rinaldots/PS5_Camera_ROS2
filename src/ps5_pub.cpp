#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <string>


class PS5PublisherNode : public rclcpp::Node {
public:
    PS5PublisherNode()
        : Node("ps5_publisher_node"), video_(2), frame_count_(0), log_interval_(5) {

        publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
        camera_info_left_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
        camera_info_right_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&PS5PublisherNode::timerCallback, this));

        // Configura câmera
        video_.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
        video_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
        video_.set(cv::CAP_PROP_FPS, 30);

        if (!video_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source.");
            rclcpp::shutdown();
        }

        loadCalibrationData();

        RCLCPP_INFO(this->get_logger(), "PS5PublisherNode initialized with 30 FPS target.");
    }

private:
    void timerCallback() {
        cv::Mat frame;
        if (!video_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from video source.");
            return;
        }

        rclcpp::Time timestamp = this->get_clock()->now();
        int mid_width = frame.cols / 2;

        cv::Mat left_frame = frame(cv::Range::all(), cv::Range(0, mid_width));
        cv::Mat right_frame = frame(cv::Range::all(), cv::Range(mid_width, frame.cols));

        // Cria mensagens de imagem
        auto msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_frame).toImageMsg();
        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_frame).toImageMsg();

        msg_left->header.stamp = timestamp;
        msg_left->header.frame_id = "left_camera";
        msg_right->header.stamp = timestamp;
        msg_right->header.frame_id = "right_camera";

        publisher_left_->publish(*msg_left);
        publisher_right_->publish(*msg_right);

        // Atualiza CameraInfo e publica
        left_camera_info_.header.stamp = timestamp;
        right_camera_info_.header.stamp = timestamp;

        camera_info_left_->publish(left_camera_info_);
        camera_info_right_->publish(right_camera_info_);

        frame_count_++;
    }

    void loadCalibrationData() {
        try {
            std::string left_calib = LEFT_CALIB_PATH;
            std::string right_calib = RIGHT_CALIB_PATH;
    
            YAML::Node left_calib_data = YAML::LoadFile(left_calib);
            YAML::Node right_calib_data = YAML::LoadFile(right_calib);
    
            // LEFT
            left_camera_info_.header.frame_id = "left_camera";
            left_camera_info_.width = left_calib_data["image_width"].as<int>();
            left_camera_info_.height = left_calib_data["image_height"].as<int>();
            left_camera_info_.distortion_model = left_calib_data["distortion_model"].as<std::string>();
    
            copyYamlArray(left_calib_data["camera_matrix"]["data"], left_camera_info_.k);
            left_camera_info_.d = left_calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();
            copyYamlArray(left_calib_data["rectification_matrix"]["data"], left_camera_info_.r);
            copyYamlArray(left_calib_data["projection_matrix"]["data"], left_camera_info_.p);
    
            // RIGHT
            right_camera_info_.header.frame_id = "right_camera";
            right_camera_info_.width = right_calib_data["image_width"].as<int>();
            right_camera_info_.height = right_calib_data["image_height"].as<int>();
            right_camera_info_.distortion_model = right_calib_data["distortion_model"].as<std::string>();
    
            copyYamlArray(right_calib_data["camera_matrix"]["data"], right_camera_info_.k);
            right_camera_info_.d = right_calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();
            copyYamlArray(right_calib_data["rectification_matrix"]["data"], right_camera_info_.r);
            copyYamlArray(right_calib_data["projection_matrix"]["data"], right_camera_info_.p);
    
            RCLCPP_INFO(this->get_logger(), "Calibration data loaded successfully.");
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration data: %s", e.what());
            rclcpp::shutdown();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    template <typename T, size_t N>
    void copyYamlArray(const YAML::Node& node, std::array<T, N>& arr) {
        auto vec = node.as<std::vector<T>>();
        if (vec.size() != N) {
            throw std::runtime_error("Size mismatch while copying YAML array to std::array");
        }
        std::copy(vec.begin(), vec.end(), arr.begin());
    }


    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_left_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_right_;

    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_;
    int frame_count_;
    const int log_interval_;

    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PS5PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
