/*
文件名: MPC具体函数的实现
作者: kq
完成时间: 2025.03.13

编译类型: 动态库

依赖: 
    外部库:
      osqp
*/

#include "mpc_controller.h"

namespace Controller
{
    MPCController::MPCController()
    {
        // 读取配置文件
    }

    /**
     * @brief 运行一次MPC控制器
     *
     * 该函数用于执行MPC控制器的一次运算周期。
     *
     * @return 返回值为布尔类型，表示MPC控制器是否成功运行一次。成功返回true，失败返回false。
     */
    bool MPCController::RunOnce()
    {
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "MPC Controller running..."); // 打印测试
        return true;
    }

    void MPCController::Initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights)
    {
    }

} // namespace Controller