#include "rclcpp/rclcpp.hpp"
#include "FlyCapture2.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "tm_robot_if/srv/second_camera.hpp"

#include <thread>
#include <mutex>
#include <filesystem>
#include <string>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <algorithm>
#include <sstream>

using namespace FlyCapture2;

class FlyCap2Node : public rclcpp::Node {
public:
    FlyCap2Node() : Node("flycap2_node"), frame_ready_(false) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/second_camera/image_raw", 10);

        // 初始化相機
        BusManager busMgr;
        unsigned int numCameras = 0;
        busMgr.GetNumOfCameras(&numCameras);
        if (numCameras == 0) {
            RCLCPP_FATAL(this->get_logger(), "No cameras found.");
            rclcpp::shutdown();
            return;
        }

        PGRGuid guid;
        busMgr.GetCameraFromIndex(0, &guid);
        cam.Connect(&guid);

        Format7ImageSettings fmt7Settings{};
        fmt7Settings.mode = MODE_0;
        fmt7Settings.offsetX = 0;
        fmt7Settings.offsetY = 0;
        fmt7Settings.width = 1920;
        fmt7Settings.height = 1080;
        fmt7Settings.pixelFormat = PIXEL_FORMAT_RAW8;

        bool valid;
        Format7PacketInfo fmt7PacketInfo;
        cam.ValidateFormat7Settings(&fmt7Settings, &valid, &fmt7PacketInfo);
        cam.SetFormat7Configuration(&fmt7Settings, fmt7PacketInfo.recommendedBytesPerPacket);
        cam.StartCapture();

        // 定時抓圖
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&FlyCap2Node::grab_frame, this)
        );

        // 延遲建立 service：等第一張圖抓到後才建立
        service_init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FlyCap2Node::maybe_create_service, this)
        );

        RCLCPP_INFO(this->get_logger(), "FlyCap2Node initialized, waiting for first frame...");
    }

    ~FlyCap2Node() {
        try {
            cam.StopCapture();
            cam.Disconnect();
        } catch (...) {}
    }

private:
    Camera cam;
    cv::Mat latest_frame_;
    std::mutex capture_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr service_init_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<tm_robot_if::srv::SecondCamera>::SharedPtr capture_service_;
    bool frame_ready_ = false;

    void grab_frame() {
        Image rawImage, rgbImage;
        if (cam.RetrieveBuffer(&rawImage) != PGRERROR_OK) return;
        rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage);
        unsigned int rowBytes = rgbImage.GetReceivedDataSize() / rgbImage.GetRows();
        latest_frame_ = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes).clone();

        // 設定 ready flag
        if (!latest_frame_.empty()) frame_ready_ = true;

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "second_camera";

        cv_bridge::CvImage bridge_msg;
        bridge_msg.header = header;
        bridge_msg.encoding = "bgr8";
        bridge_msg.image = latest_frame_;
        image_pub_->publish(*bridge_msg.toImageMsg());
    }

    void maybe_create_service() {
        if (!frame_ready_) return;
        service_init_timer_->cancel();

        capture_service_ = this->create_service<tm_robot_if::srv::SecondCamera>(
            "camera2",
            std::bind(&FlyCap2Node::capture_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "First frame received. Service 'camera2' ready.");
    }

    void capture_callback(
        const std::shared_ptr<tm_robot_if::srv::SecondCamera::Request> request,
        std::shared_ptr<tm_robot_if::srv::SecondCamera::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "SecondCamera service called. Waiting for 4 signals...");

        int count = 0;
        std::string dir = std::filesystem::absolute("./src/second_camera/sample_picture/").string();
        std::filesystem::create_directories(dir);

        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(6000);
        addr.sin_addr.s_addr = INADDR_ANY;
        bind(server_fd, (struct sockaddr*)&addr, sizeof(addr));
        listen(server_fd, 1);
        RCLCPP_INFO(this->get_logger(), "Socket server listening on port 6000");

        int client_fd = accept(server_fd, nullptr, nullptr);
        RCLCPP_INFO(this->get_logger(), "Socket client connected.");

        char buf[1024];
        while (count < 4) {
            ssize_t bytes = recv(client_fd, buf, sizeof(buf), 0);
            if (bytes <= 0) break;

            std::ostringstream raw_stream;
            for (ssize_t i = 0; i < bytes; ++i)
                raw_stream << std::hex << std::showbase << (int)(unsigned char)buf[i] << " ";
            RCLCPP_INFO(this->get_logger(), "[RAW] %ld bytes: %s", bytes, raw_stream.str().c_str());

            std::string msg(buf, bytes);
            msg.erase(std::remove_if(msg.begin(), msg.end(), ::isspace), msg.end());
            std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);

            bool is_true = false;
            if (bytes == 1 && ((unsigned char)buf[0] == 0x01 || buf[0] == '1')) {
                is_true = true;
            } else if (msg == "true") {
                is_true = true;
            }

            if (is_true) {
                RCLCPP_INFO(this->get_logger(), "Received TRUE signal %d/4", count + 1);

                Image rawImage, rgbImage;
                if (cam.RetrieveBuffer(&rawImage) == PGRERROR_OK) {
                    rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage);
                    unsigned int rowBytes = rgbImage.GetReceivedDataSize() / rgbImage.GetRows();
                    cv::Mat current_frame(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);

                    std::string filename = dir + "img" + std::to_string(count + 1) + ".jpg";
                    if (cv::imwrite(filename, current_frame)) {
                        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.c_str());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to grab frame on demand.");
                }

                count++;
            } else {
                RCLCPP_WARN(this->get_logger(), "Unrecognized message: '%s'", msg.c_str());
            }
        }

        close(client_fd);
        close(server_fd);

        combine_images(dir, dir + "/combined.jpg");

        RCLCPP_INFO(this->get_logger(), "Done capturing and combining images.");
        response->success = true;
    }

    void combine_images(const std::string& input_dir, const std::string& output_path) {
        std::vector<std::string> filenames = { "img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg" };
        std::vector<cv::Mat> images;
        for (const auto& name : filenames) {
            cv::Mat img = cv::imread(input_dir + "/" + name);
            if (img.empty()) {
                RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", name.c_str());
                return;
            }
            cv::resize(img, img, cv::Size(1024, 1024));
            images.push_back(img);
        }

        if (images.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Missing images. Skipping combination.");
            return;
        }

        cv::Mat top, bottom, combined;
        cv::hconcat(images[0], images[1], top);
        cv::hconcat(images[2], images[3], bottom);
        cv::vconcat(top, bottom, combined);
        cv::imwrite(output_path, combined);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlyCap2Node>());
    rclcpp::shutdown();
    return 0;
}
