#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class UsbCameraPublisher : public rclcpp::Node
{
public:
    UsbCameraPublisher() : Node("usb_camera_publisher"), camera_ready_(false)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&UsbCameraPublisher::timer_callback, this));

        // Try to open camera from device 0 to 5
        for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(this->get_logger(), "Attempting to open camera device %d...", i);
            cap_.open(i);
            if (cap_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "Successfully opened camera device %d", i);
                camera_ready_ = true;
                return;
            }
        }
        RCLCPP_WARN(this->get_logger(), "Could not open any camera device. Continuing anyway - will retry on next frame.");
    }

private:
    void timer_callback()
    {
        // Try to open camera if not already open
        if (!camera_ready_ && !cap_.isOpened()) {
            for (int i = 0; i < 6; ++i) {
                cap_.open(i);
                if (cap_.isOpened()) {
                    RCLCPP_INFO(this->get_logger(), "Successfully opened camera device %d", i);
                    camera_ready_ = true;
                    break;
                }
            }
            if (!camera_ready_) {
                return;  // Still no camera, try again next time
            }
        }

        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Captured empty frame");
            return;
        }

        // Convert OpenCV image to ROS2 image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";

        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    bool camera_ready_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsbCameraPublisher>());
    rclcpp::shutdown();
    return 0;
}