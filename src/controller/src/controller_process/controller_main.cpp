/*
文件名: 控制总流程入口
作者: kq
完成时间: 2025.03.11

编译类型: 节点(controller_process)

流程: 为节点controller_process提供入口函数main()
*/

#include "controller_process.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("controller_process_main"), "controller start");

  auto node = std::make_shared<Controller::ControllerProcess>();
  if(!node->MainLoop())
  {
    RCLCPP_ERROR(rclcpp::get_logger("controller_process_main"), "controller failed!");
    rclcpp::shutdown();
    return 1;
  }

  // RCLCPP_INFO(rclcpp::get_logger("controller_process_main"), "controller start spinning");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}