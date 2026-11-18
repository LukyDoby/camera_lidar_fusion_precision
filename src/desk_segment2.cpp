#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageProcessorNode : public rclcpp::Node
{
public:
    ImageProcessorNode() : Node("image_processor_node")
    {
        // Subscribe to incoming images
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));

        // Publisher for processed images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_desk", 10);

        RCLCPP_INFO(this->get_logger(), "Image Processor Node has started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {   RCLCPP_INFO(this->get_logger(), "Image received!");
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            cv::Mat labImage;
            cv::cvtColor(cv_ptr->image, labImage, cv::COLOR_BGR2Lab);

            // Split the channels
            std::vector<cv::Mat> labChannels(3);
            cv::split(labImage, labChannels);

            // Extract L, a, b channels
            cv::Mat L = labChannels[0];  // Lightness
            cv::Mat a = labChannels[1];  // a-channel
            cv::Mat b = labChannels[2];  // b-channel

            // Create binary mask
            cv::Mat mask = (L >= L_min) & (L <= L_max)&
                (a >= a_min) & (a <= a_max);
            // (b >= b_min) & (b <= b_max);

            // // Apply morphological operation (dilation in this case)

            // cv::Mat processed_image;
            // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            // cv::morphologyEx(mask, processed_image, cv::MORPH_CLOSE, kernel);

            // Convert back to ROS image message
            cv_bridge::CvImage processed_msg;
            processed_msg.header = msg->header;
            processed_msg.encoding = "mono8";
            processed_msg.image = mask;

            // Publish the processed image
            publisher_->publish(*processed_msg.toImageMsg());

            RCLCPP_INFO(this->get_logger(), "Published processed image.");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    int L_min = 135;
    int L_max = 221;
    int a_min = 149;
    int a_max = 255;
    int b_min = 0;  // OpenCV stores "a" and "b" values shifted, so check next note.
    int b_max = 255;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
