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
        mpc_config_ = std::make_unique<ConfigReader>();
        mpc_config_->read_mpc_param();
        MpcParamStruct params;
        params = mpc_config_->mpc_param();
        mpc_config_->read_mpc_model();
        MpcModelStruct model;
        model = mpc_config_->mpc_model();
        mpc_config_->read_mpc_hard_constraint();
        MpcHardConstraintStruct hard_constraint;
        hard_constraint = mpc_config_->mpc_hard_constraint();
        mpc_config_->read_mpc_cost_weight();
        MpcCostWeightStruct cost_weight;
        cost_weight = mpc_config_->mpc_cost_weight();

        // 将配置参数赋值给成员变量
        Initialize(params, model, hard_constraint, cost_weight);
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
        // LogTest();
        return true;
    }

    void MPCController::Initialize(const MpcParamStruct &parameters, const MpcModelStruct &model, const MpcHardConstraintStruct &constraint, const MpcCostWeightStruct &weights)
    {
        // 参数
        param_.dt = parameters.dt_;
        param_.pred_horizon = parameters.pred_horizon_;
        param_.control_horizon = parameters.control_horizon_;

        // 动力学模型
        model_.l_f = model.lf_;
        model_.l_r = model.lr_;
        model_.m = model.m_;

        // 硬约束
        hard_constraint_.max_steer = constraint.max_steer_;
        hard_constraint_.max_accel = constraint.max_acceleration_;
        hard_constraint_.min_accel = constraint.min_acceleration_;
        hard_constraint_.max_steer_rate = constraint.max_steer_rate_;
        hard_constraint_.max_jerk = constraint.max_jerk_;

        // 代价权重
        cost_weight_.w_position = weights.w_position_;
        cost_weight_.w_angle = weights.w_heading_;
        cost_weight_.w_velocity = weights.w_velocity_;
        cost_weight_.w_steer = weights.w_steer_;
        cost_weight_.w_accel = weights.w_acceleration_;
        cost_weight_.w_dsteer = weights.w_steer_rate_;
        cost_weight_.w_jerk = weights.w_jerk_;
    }

    void MPCController::LogTest()
    {
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "prediction horizon: %d", param_.pred_horizon);
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "hard_constraint max_steer: %f", hard_constraint_.max_steer);
    }

    double MPCController::NormalizeAngle(double a)
    {
        return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
    }

} // namespace Controller