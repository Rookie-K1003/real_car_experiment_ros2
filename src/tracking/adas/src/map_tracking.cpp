#include "control.h"
#include "plan.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(FRE);

    auto adas_planer = std::make_shared<adas_plan>(rclcpp::NodeOptions{});
    auto adas_controler = std::make_shared<adas_control>(rclcpp::NodeOptions{});

    while(rclcpp::ok())
    {
        adas_controler->cal_control(adas_planer->car_, adas_planer->route_data_);
        adas_controler->send_control();

        adas_planer->visulization();

        rclcpp::spin_some(adas_planer);
        rclcpp::spin_some(adas_controler);
        loop_rate.sleep();
    }
}