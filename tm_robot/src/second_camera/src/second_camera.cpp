#include "rclcpp/rclcpp.hpp"
#include "FlyCapture2.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <filesystem>

#include "tm_robot_if/srv/second_camera.hpp"

using namespace FlyCapture2;
using std::placeholders::_1;
using std::placeholders::_2;

class FlyCap2Node : public rclcpp::Node
{
public:
    FlyCap2Node() : Node("flycap2_stream_node")
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to camera...");

        // === 建立 ROS 2 service ===
        capture_service_ = this->create_service<tm_robot_if::srv::SecondCamera>(
            "/SecondCamera/capture",
            std::bind(&FlyCap2Node::capture_callback, this, _1, _2)
        );

        // === 建立 ROS 2 publisher ===
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/second_camera/image_raw", 10);

        // === 相機初始化 ===
        BusManager busMgr;
        unsigned int numCameras = 0;
        Error error = busMgr.GetNumOfCameras(&numCameras);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "GetNumOfCameras failed: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Detected cameras: %u", numCameras);

        if (numCameras == 0) {
            RCLCPP_FATAL(this->get_logger(), "No cameras detected.");
            rclcpp::shutdown();
            return;
        }

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(0, &guid);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get camera from index 0: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        error = cam.Connect(&guid);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Camera connect failed: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        // === 設定 Format7 模式 ===
        Format7ImageSettings fmt7Settings = {};
        fmt7Settings.mode = MODE_0;
        fmt7Settings.offsetX = 0;
        fmt7Settings.offsetY = 0;
        fmt7Settings.width = 1280;
        fmt7Settings.height = 720;
        fmt7Settings.pixelFormat = PIXEL_FORMAT_BGR;

        bool valid;
        Format7PacketInfo fmt7PacketInfo;
        error = cam.ValidateFormat7Settings(&fmt7Settings, &valid, &fmt7PacketInfo);
        if (!valid || error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Invalid Format7 settings: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        error = cam.SetFormat7Configuration(&fmt7Settings, fmt7PacketInfo.recommendedBytesPerPacket);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Format7 config: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        // === 啟動擷取 ===
        error = cam.StartCapture();
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Start capture failed: %s", error.GetDescription());
            rclcpp::shutdown();
            return;
        }

        // === 建立定時擷取 timer ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&FlyCap2Node::grab_frame, this)
        );

        RCLCPP_INFO(this->get_logger(), "Camera ready. Service [/SecondCamera/capture] and topic [/second_camera/image_raw] available.");

        rclcpp::on_shutdown([this]() {
            this->shutdown_camera();
        });
    }

    ~FlyCap2Node()
    {
        shutdown_camera();
    }

private:
    Camera cam;
    cv::Mat latest_frame_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<tm_robot_if::srv::SecondCamera>::SharedPtr capture_service_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    void grab_frame()
    {
        Image rawImage, rgbImage;
        if (cam.RetrieveBuffer(&rawImage) != PGRERROR_OK) {
            RCLCPP_WARN(this->get_logger(), "Frame drop.");
            return;
        }

        rawImage.Convert(PIXEL_FORMAT_BGR, &rgbImage);
        unsigned int rowBytes = rgbImage.GetReceivedDataSize() / rgbImage.GetRows();
        latest_frame_ = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes).clone();

        // 發布 ROS 2 影像
        if (!latest_frame_.empty()) {
            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            header.frame_id = "second_camera";

            cv_bridge::CvImage bridge_msg;
            bridge_msg.header = header;
            bridge_msg.encoding = "bgr8";
            bridge_msg.image = latest_frame_;

            image_pub_->publish(*bridge_msg.toImageMsg());
        }
    }

    void capture_callback(
        const std::shared_ptr<tm_robot_if::srv::SecondCamera::Request> request,
        std::shared_ptr<tm_robot_if::srv::SecondCamera::Response> response)
    {
        if (latest_frame_.empty()) {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "No frame available to save.");
            return;
        }

        std::string dir = "/workspace/tm_robot/data/second_cam/";
        std::filesystem::create_directories(dir);
        std::string path = dir + request->filename;

        if (cv::imwrite(path, latest_frame_)) {
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Saved image to: %s", path.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", path.c_str());
        }
    }

    void shutdown_camera()
    {
        try {
            cam.StopCapture();
            cam.Disconnect();
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Failed to safely stop or disconnect camera.");
        }
        // 已移除 cv::destroyAllWindows()
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FlyCap2Node>());
    } catch (const std::exception& e) {
        std::cerr << "Node failed: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
