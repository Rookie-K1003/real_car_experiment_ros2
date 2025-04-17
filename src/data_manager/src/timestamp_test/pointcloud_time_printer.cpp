#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudTimePrinter : public rclcpp::Node
{
public:
    PointCloudTimePrinter()
    : Node("pointcloud_time_printer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10,
            std::bind(&PointCloudTimePrinter::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 获取时间戳
        uint32_t sec = msg->header.stamp.sec;
        uint32_t nanosec = msg->header.stamp.nanosec;
        double timestamp = sec + nanosec * 1e-9;

        // 打印到终端
        // RCLCPP_INFO(this->get_logger(), "Received PointCloud2:");
        RCLCPP_INFO(this->get_logger(), "  sec      = %u", sec);
        RCLCPP_INFO(this->get_logger(), "  nanosec  = %u", nanosec);
        RCLCPP_INFO(this->get_logger(), "  timestamp= %.9f", timestamp);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTimePrinter>());
    rclcpp::shutdown();
    return 0;
}