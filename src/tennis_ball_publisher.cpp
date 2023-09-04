#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"

class TennisBallPublisher : public rclcpp::Node
{
public:
    TennisBallPublisher() : Node("tennis_ball_publisher")
    {
        auto videoPath = ament_index_cpp::get_package_share_directory("tennis_ball_tracker") + "/resources/tennis-ball-video.mp4";
        m_videoCapture.open(videoPath);
        if (!m_videoCapture.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the video file");
            rclcpp::shutdown();
            return;
        }
        m_ballImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/tennis_ball_image", 10);

        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&TennisBallPublisher::TimerCallBack, this));
    }

    ~TennisBallPublisher()
    {
        m_videoCapture.release();
    }

private:
    void TimerCallBack()
    {
        cv::Mat frame;
        m_videoCapture >> frame;
        if (!frame.empty())
        {
            // Convert the OpenCV frame to a ROS image message
            sensor_msgs::msg::Image image_msg;
            image_msg.header.stamp = now();
            image_msg.header.frame_id = "camera_frame";
            image_msg.height = frame.rows;
            image_msg.width = frame.cols;
            image_msg.encoding = "bgr8";
            image_msg.is_bigendian = false;
            image_msg.step = frame.cols * frame.elemSize();
            image_msg.data.assign(frame.data, frame.data + frame.total() * frame.elemSize());

            // Publish the image message
            m_ballImagePublisher->publish(image_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Video ended. Shutting down...");
            rclcpp::shutdown();
        }
    }
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_ballImagePublisher;
    cv::VideoCapture m_videoCapture;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TennisBallPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}