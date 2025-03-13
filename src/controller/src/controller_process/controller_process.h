#ifndef CONTROLLER_PROCESS_H_
#define CONTROLLER_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"

#include <vector>
#include <cmath>
#include <algorithm>

// 用到的msg和srv类型
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"

namespace Controller
{
    using base_msgs::srv::GlobalPathService;
    using nav_msgs::msg::Path;
    using namespace std::chrono_literals;
    
    class ControllerProcess : public rclcpp::Node // 控制总流程
    {
    public:
        ControllerProcess();
        bool MainLoop(); // 控制器主循环
    
    private:
        bool ControllerInit(); // 控制器初始化

    public:
        inline Path GetGlobalPath() const { return global_path_; } // 用于其他类获取全局路径        

    private:
        std::unique_ptr<ConfigReader> process_config_; // 用于配置文件读取
        
        Path global_path_; // 全局路径
        // rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径请求客户端  ！废弃，改用订阅全局路径话题
        // *订阅者
        rclcpp::Subscription<Path>::SharedPtr global_path_sub_;
        // *回调函数
        void GlobalPathCallback(const Path::ConstSharedPtr msg);

    };

} // namespace Controller
#endif // CONTROLLER_PROCESS_H_