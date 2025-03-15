#ifndef CONTROLLER_PROCESS_H_
#define CONTROLLER_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "mpc_controller.h"

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
        bool MainThread(); // 控制器主线程
        void MPCControlCycleCallback(); // MPC控制周期回调函数    
    
    private:
        bool ControllerInit(); // 控制器初始化

    public:
        inline Path GetGlobalPath() const { return global_path_; } // 用于其他类获取全局路径        

    private:
        std::unique_ptr<ConfigReader> process_config_; // 用于配置文件读取
        
        Path global_path_; // 全局路径

        // ------ros客户端与服务端通信相关------
        // rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径请求客户端  ！废弃，改用订阅全局路径话题
        // *订阅者
        rclcpp::Subscription<Path>::SharedPtr global_path_sub_;
        // TBD：用于rviz+urdf仿真的车辆状态如何订阅??

        // *回调函数
        void GlobalPathCallback(const Path::ConstSharedPtr msg);
    
    private:
        // *不同控制器对象
        MPCController mpc_controller_; // MPC控制器，用于调用其函数

    };

} // namespace Controller
#endif // CONTROLLER_PROCESS_H_