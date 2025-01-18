#include "rclcpp/rclcpp.hpp"
#include "location_msgs/msg/rtk.hpp"

class debug : public rclcpp::Node
{
    public:
        explicit debug(const rclcpp::NodeOptions & node_options) : Node("debug", node_options) 
        {
            sub = this->create_subscription<location_msgs::msg::RTK>("Huace_rtk", 1, std::bind(&debug::recvHuacePosCallback, this, std::placeholders::_1));
        }

    private:
        void recvHuacePosCallback(const location_msgs::msg::RTK::ConstSharedPtr msg) 
        {
            RCLCPP_INFO(this->get_logger(), "longitude: %.8lf, latitude : %.8lf", msg->longitude, msg->latitude);
        }
        rclcpp::Subscription<location_msgs::msg::RTK>::SharedPtr sub;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<debug>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}