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
    using nav_msgs::msg::Path;

    class MPCController: public rclcpp::Node
    {
    public:
        MPCController();
        
        bool RunOnce(); // MPC控制器一次计算过程

        void Initialize(const MpcParamStruct &parameters, const MpcModelStruct &model, 
                        const MpcHardConstraintStruct &constraint, 
                        const MpcCostWeightStruct &weights);
        
        void LogTest(); // 测试日志输出
        double NormalizeAngle(double a);
        State GetStateAndLastControl(); // 获取状态量和上一时刻控制量
        PoseStamped GetStartPose(); // 获取起始位姿

        void Update(const State &state, const State &linearize_point, 
                    const std::vector<PoseStamped> &track_input, 
                    ControlOutput *out, std::vector<State> *pred_out); // 计算MPC矩阵的核心部分
        
        void ControllerModelSimulate(const State &state, State *next);
        void IterativeUpdate(const State &state, const std::vector<PoseStamped> &track_input, 
                            int iterations, double threshold, 
                            ControlOutput *out, std::vector<State> *pred_out); // 迭代更新，用于求解最优控制量
        bool GetReferenceTrajectory(Path ref_path); // 获取参考轨迹        

        
    private:
        std::unique_ptr<ConfigReader> mpc_config_; // 用于配置文件读取
        std::unique_ptr<PathAdapter> path_adapter_;
        QPProblem <c_float> qp_;

        VehicleStateMPC state_; // 用于状态量记录
        double last_steer_angle_; // 用于记录上一时刻的转向角
        double last_accel_; // 用于记录上一时刻的加速度
        Path ref_path_; // 用于存放参考轨迹

        // * mpc_tools.h中定义的参数结构体，用于存放MPC的参数信息
        Parameters param_; // 用于参数配置
        Model model_; // 用于模型配置
        HardConstraint hard_constraint_; // 用于硬约束配置
        CostFunctionWeights cost_weight_; // 用于代价配置

        rclcpp::Subscription<Path>::SharedPtr ref_path_sub_;
        // *回调函数
        void RefPathCallback(const Path::ConstSharedPtr msg);

    };

} // namespace Controller

#endif // MPC_CONTROLLER_H__