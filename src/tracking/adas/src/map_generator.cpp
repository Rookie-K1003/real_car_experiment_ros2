#include "rtk_map.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto map_ = std::make_shared<rtk_map>(rclcpp::NodeOptions{});
}