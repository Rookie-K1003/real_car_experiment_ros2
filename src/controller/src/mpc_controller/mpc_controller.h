#ifndef MPC_CONTROLLER_H__
#define MPC_CONTROLLER_H__

#include "path_adapter.h"
#include "osqp.h"
#include <vector>
#include "mpc_tools.h"
#include "config_reader.h"
#include "rclcpp/rclcpp.hpp"


#include "nav_msgs/msg/path.hpp"
#include "base_msgs/msg/vehicle_state_mpc.hpp"

namespace Controller
{
    using base_msgs::msg::VehicleStateMPC;
    class MPCController
    {
    public:
        MPCController();
        
        bool RunOnce();
        void Initialize(const Parameters &parameters, const Model &model, 
                        const HardConstraint &constraint, const CostFunctionWeights &weights);
        
    private:
        std::unique_ptr<ConfigReader> mpc_config_; // 用于配置文件读取
        VehicleStateMPC state_; // 用于状态量计算

        // mpc_tools.h中定义的参数结构体，用于存放MPC的参数信息
        Parameters params_; // 用于参数配置
        Model model_; // 用于模型配置
        HardConstraint hard_constraint_; // 用于硬约束配置
        CostFunctionWeights cost_weights_; // 用于代价配置

    };

} // namespace Controller

#endif // MPC_CONTROLLER_H__