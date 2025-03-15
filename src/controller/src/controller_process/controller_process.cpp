/*
文件名: 控制总流程
作者: kq
完成时间: 2025.03.11

编译类型: 节点(controller_process)

依赖: ROS2内部库:
      rclcpp
      nav_msgs
      tf2
      tf2_ros
    外部库:
      base_msgs
      config_reader
      basic_controller：基础控制器：纵向PID控制、横向LQR控制
      mpc_controller
*/

#include "controller_process.h"

namespace Controller
{
    ControllerProcess::ControllerProcess() : Node("controller_process") // 控制总流程
    {
        RCLCPP_INFO(this->get_logger(), "controller_process created");

        // 读取配置文件
        process_config_ = std::make_unique<ConfigReader>();

        // // 订阅参考轨迹，暂时用全局路径代替
        // global_path_sub_ = this->create_subscription<Path>(
        //     "/planning/global_path_planning", rclcpp::SensorDataQoS(), std::bind(&ControllerProcess::GlobalPathCallback, this, std::placeholders::_1));

    }

    bool ControllerProcess::MainThread()
    {
        // 进入控制主流程
        RCLCPP_INFO(this->get_logger(), "ControllerProcess MainLoop Start!");

        rclcpp::Rate rate(10.0); 
        // 循环
        while (rclcpp::ok())
        {
            // TODO: 判断使用哪种控制器
            MPCControlCycleCallback();

            rate.sleep();
        }

        return false;
    }

    void ControllerProcess::MPCControlCycleCallback()
    {
        // ~ step1: 获取参考路径/轨迹，不满足xx条件，直接返回
        // TODO：求解失败、未求解完成...
        // if (!mpc_controller_.GetReferenceTrajectory(global_path_)) {
        //     RCLCPP_ERROR(this->get_logger(), "[MPC_Controller] Fail to get ref_traj info");
        //     return;
        // }

        // ~ step2: 控制器运行
        if (!mpc_controller_.RunOnce()) {
            RCLCPP_ERROR(this->get_logger(), "MPC controller failed");
            return;
        }

        // ~ step3: 发布数据

        // ~ step4: 控制器状态更新

        // ~ step5: 可视化
        
    }

    // !!暂时先不用
    bool ControllerProcess::ControllerInit()
    {
        // 如果参考轨迹为空，则控制器初始化失败 ！！需要先启动规划线程
        // 这里参考轨迹先用全局路径代替
        // 等待几秒，让参考轨迹订阅完成
        rclcpp::Rate rate0(3);
        rate0.sleep();
        
        if (global_path_.poses.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Global path is empty");
            return false;
        }
        
        return true;
    }

    void ControllerProcess::GlobalPathCallback(const Path::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received global path");
        // 更新全局路径
        global_path_ = *msg;

        // 打印，用于调试
        // RCLCPP_INFO(this->get_logger(), "Global path size: %ld", global_path_.poses.size());
        // // 打印全局路径坐标
        // for (int i = 0; i < global_path_.poses.size(); i++)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Global path %d: x=%f, y=%f", i, global_path_.poses[i].pose.position.x,
        //                 global_path_.poses[i].pose.position.y);
        // }
    }

} // namespace Controller