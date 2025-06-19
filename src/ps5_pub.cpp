#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.hpp>
#include <array>
#include <string>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <thread>
#include <chrono>


class PS5PublisherNode : public rclcpp::Node {
public:
    PS5PublisherNode()
        : Node("ps5_publisher_node"), log_interval_(5) {

        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));

        // Detecta automaticamente a câmera PS5
        int camera_index = detectPS5Camera();
        if (camera_index == -1) {
            RCLCPP_ERROR(this->get_logger(), "PS5 camera not found!");
            rclcpp::shutdown();
            return;
        }

        video_.open(camera_index);
        RCLCPP_INFO(this->get_logger(), "PS5 camera detected at index %d", camera_index);

        // Usa image_transport::create_camera_publisher
        camera_pub_left_ = image_transport::create_camera_publisher(this, "left/image_raw", qos_profile.get_rmw_qos_profile());
        camera_pub_right_ = image_transport::create_camera_publisher(this, "right/image_raw", qos_profile.get_rmw_qos_profile());
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
        left_camera_info_.header.stamp = timestamp;
        right_camera_info_.header.stamp = timestamp;

        // Publica imagem e camera_info usando CameraPublisher
        // O CameraPublisher irá definir o timestamp do CameraInfo para corresponder ao da imagem.
        camera_pub_left_.publish(*msg_left, left_camera_info_);
        camera_pub_right_.publish(*msg_right, right_camera_info_);
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

    int detectPS5Camera() {
        const std::string PS5_VENDOR_ID = "05a9";
        const std::string PS5_PRODUCT_ID_INITIAL = "0580";  // Before firmware
        const std::string PS5_PRODUCT_ID_LOADED = "058c";   // After firmware
        
        RCLCPP_INFO(this->get_logger(), "Searching for PS5 camera...");
        
        // First check if camera with firmware is already available
        for (int i = 0; i < 10; ++i) {
            if (isPS5Camera(i, PS5_VENDOR_ID, PS5_PRODUCT_ID_LOADED)) {
                cv::VideoCapture test_cap(i);
                if (test_cap.isOpened()) {
                    test_cap.release();
                    RCLCPP_INFO(this->get_logger(), "Found PS5 camera with firmware at video%d", i);
                    return i;
                }
            }
        }
        
        // Check if camera without firmware is present and load firmware
        if (checkAndLoadFirmware(PS5_VENDOR_ID, PS5_PRODUCT_ID_INITIAL)) {
            // Wait a bit for the device to reinitialize after firmware loading
            RCLCPP_INFO(this->get_logger(), "Waiting for camera to reinitialize...");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Try again to find the camera with firmware loaded
            for (int i = 0; i < 10; ++i) {
                if (isPS5Camera(i, PS5_VENDOR_ID, PS5_PRODUCT_ID_LOADED)) {
                    cv::VideoCapture test_cap(i);
                    if (test_cap.isOpened()) {
                        test_cap.release();
                        RCLCPP_INFO(this->get_logger(), "Found PS5 camera at video%d after firmware loading", i);
                        return i;
                    }
                }
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "PS5 camera not found or firmware loading failed.");
        return -1;
    }

    bool checkAndLoadFirmware(const std::string& vendor_id, const std::string& product_id) {
        // Check if camera without firmware is present via lsusb
        std::string lsusb_cmd = "lsusb | grep " + vendor_id + ":" + product_id + " > /dev/null 2>&1";
        int result = std::system(lsusb_cmd.c_str());
        
        if (result == 0) {  // Camera found without firmware
            RCLCPP_INFO(this->get_logger(), "PS5 camera found without firmware. Loading firmware...");
            
            // Try common paths for the firmware loader
            std::vector<std::string> possible_paths = {
                "/home/r1/PS5-Camera-Firmware-Loader/cpp/ps5_camera_firmware_loader",
                "/home/orangepi/PS5-Camera-Firmware-Loader/cpp/ps5_camera_firmware_loader",
                "./ps5_camera_firmware_loader",
                "ps5_camera_firmware_loader"
            };
            
            std::vector<std::string> possible_firmware = {
                "/home/r1/PS5-Camera-Firmware-Loader/cpp/firmware.bin",
                "/home/orangepi/PS5-Camera-Firmware-Loader/cpp/firmware.bin",
                "./firmware.bin",
                "firmware.bin"
            };
            
            for (const auto& loader_path : possible_paths) {
                if (std::filesystem::exists(loader_path)) {
                    for (const auto& firmware_path : possible_firmware) {
                        if (std::filesystem::exists(firmware_path)) {
                            std::string cmd = loader_path + " " + firmware_path + " > /dev/null 2>&1";
                            RCLCPP_INFO(this->get_logger(), "Executing: %s %s", loader_path.c_str(), firmware_path.c_str());
                            
                            int firmware_result = std::system(cmd.c_str());
                            if (firmware_result == 0) {
                                RCLCPP_INFO(this->get_logger(), "Firmware loaded successfully!");
                                return true;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Firmware loading failed with exit code: %d", firmware_result);
                            }
                        }
                    }
                }
            }
            
            RCLCPP_ERROR(this->get_logger(), "Could not find firmware loader or firmware file!");
            return false;
        }
        
        return false;  // Camera not found without firmware
    }

    bool isPS5Camera(int index, const std::string& vendor_id, const std::string& product_id) {
        std::string device_path = "/sys/class/video4linux/video" + std::to_string(index) + "/device";
        
        if (!std::filesystem::exists(device_path)) {
            return false;
        }
        
        try {
            // Navega até o dispositivo USB
            std::filesystem::path usb_device_path = std::filesystem::canonical(device_path);
            
            // Procura pelo diretório USB que contém idVendor e idProduct
            while (usb_device_path != usb_device_path.root_path()) {
                std::filesystem::path vendor_file = usb_device_path / "idVendor";
                std::filesystem::path product_file = usb_device_path / "idProduct";
                
                if (std::filesystem::exists(vendor_file) && std::filesystem::exists(product_file)) {
                    std::ifstream vendor_stream(vendor_file);
                    std::ifstream product_stream(product_file);
                    
                    std::string device_vendor, device_product;
                    if (vendor_stream >> device_vendor && product_stream >> device_product) {
                        RCLCPP_DEBUG(this->get_logger(), "video%d: vendor=%s, product=%s", 
                                   index, device_vendor.c_str(), device_product.c_str());
                        
                        if (device_vendor == vendor_id && device_product == product_id) {
                            return true;
                        }
                    }
                    break;
                }
                usb_device_path = usb_device_path.parent_path();
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "Error checking device %d: %s", index, e.what());
        }
        
        return false;
    }

    image_transport::CameraPublisher camera_pub_left_;
    image_transport::CameraPublisher camera_pub_right_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_;
    const int log_interval_;

    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PS5PublisherNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
