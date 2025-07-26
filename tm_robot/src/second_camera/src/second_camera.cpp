#include "rclcpp/rclcpp.hpp"
#include "FlyCapture2.h"
#include <opencv2/opencv.hpp>
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

        // 建立 service
        capture_service_ = this->create_service<tm_robot_if::srv::SecondCamera>(
            "/SecondCamera/capture",
            std::bind(&FlyCap2Node::capture_callback, this, _1, _2)
        );

        // 初始化相機
        BusManager busMgr;
        PGRGuid guid;
        Error error = busMgr.GetCameraFromIndex(0, &guid);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "No camera detected!");
            rclcpp::shutdown();
            return;
        }

        error = cam.Connect(&guid);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Camera connect failed.");
            rclcpp::shutdown();
            return;
        }

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
            RCLCPP_FATAL(this->get_logger(), "Invalid Format7 settings.");
            rclcpp::shutdown();
            return;
        }

        error = cam.SetFormat7Configuration(&fmt7Settings, fmt7PacketInfo.recommendedBytesPerPacket);
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Format7 configuration.");
            rclcpp::shutdown();
            return;
        }

        error = cam.StartCapture();
        if (error != PGRERROR_OK) {
            RCLCPP_FATAL(this->get_logger(), "Start capture failed.");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&FlyCap2Node::grab_frame, this)
        );

        RCLCPP_INFO(this->get_logger(), "Camera ready. Service [/SecondCamera/capture] available.");
    }

    ~FlyCap2Node()
    {
        cam.StopCapture();
        cam.Disconnect();
        cv::destroyAllWindows();
    }

private:
    Camera cam;
    cv::Mat latest_frame_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<tm_robot_if::srv::SecondCamera>::SharedPtr capture_service_;

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

        cv::imshow("FlyCapture2 Stream", latest_frame_);
        cv::waitKey(1);
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
        std::filesystem::create_directories(dir);  // 建立資料夾
        std::string path = dir + request->filename;

        if (cv::imwrite(path, latest_frame_)) {
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Saved image to: %s", path.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), " Failed to save image to: %s", path.c_str());
        }
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
