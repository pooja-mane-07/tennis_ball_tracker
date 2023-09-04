#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>

class TennisBallListener : public rclcpp::Node
{

public:
    TennisBallListener() : Node("tennis_ball_listener")
    {
        m_ballImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "tennis_ball_image", 10, std::bind(&TennisBallListener::ImageCallback, this, std::placeholders::_1));
    }

    ~TennisBallListener()
    {
        cv::destroyAllWindows();
    }

private:
    void ImageCallback(sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert the ROS image message to an OpenCV image
            cv::Mat frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
            DetectTennisBall(frame);
            cv::waitKey(1);
        }
        catch (cv::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void DetectTennisBall(cv::Mat &frame)
    {
        cv::Mat hsvFrame;
        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
        // define upper and lower color range
        cv::Scalar lowerYellow(30, 150, 100);
        cv::Scalar upperYellow(60, 255, 255);

        cv::Mat yellowMask;
        cv::inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        cv::imshow("Mask", yellowMask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellowMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 2500)
            {
                cv::Moments moment = cv::moments(contour);
                cv::Point2f centroid(moment.m10 / moment.m00, moment.m01 / moment.m00);
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);
                cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 2);
                cv::imshow("Contour", frame);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_ballImageSubscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TennisBallListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
