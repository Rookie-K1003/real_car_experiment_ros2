#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

namespace Controller
{
    struct VehicleStruct // 车辆(几何、运动学参数、仿真可视化)
    {
        int id_ = 0;              // 序号
        std::string frame_ = "";  // 坐标系
        double length_ = 0.0;     // 长
        double width_ = 0.0;      // 宽
        double pose_x_ = 0.0;     // x
        double pose_y_ = 0.0;     // y
        double pose_theta_ = 0.0; // 航向角
        double speed_ori_ = 0.0;  // 初速度
    };

    struct GlobalPathStruct // 全局路径
    {
        int type_ = 0; // 类型:直线、S弯道等
    };

    struct ControllerTypeStruct
    {
        int type_ = 0; // 类型
    };
    
    struct PidParamStruct
    {
        double kp_ = 0.0;
        double ki_ = 0.0;
        double kd_ = 0.0;
    };

    struct LqrDynamicParamStruct
    {
        double Q_ed_ = 0.0;
        double Q_ed_dot_ = 0.0;
        double Q_ephi_ = 0.0;
        double Q_ephi_dot_ = 0.0;
        double R_value_ = 0.0;
    };

    struct LqrKinematicParamStruct
    {
        double Q_ex_k_ = 0.0;
        double Q_ed_k_ = 0.0;
        double Q_ephi_k_ = 0.0;
        double R_value_k_ = 0.0;
    };

    struct MpcParamStruct
    {
        double dt_ = 0.0;
        int pred_horizon_ = 0;
        int control_horizon_ = 0;
    };

    struct MpcModelStruct // 根据要选择的不同动力学模型，进行设置
    {
        double lf_ = 0.0;
        double lr_ = 0.0;
        double m_ = 0.0;
        double Iz_ = 0.0;    
    };

    struct MpcHardConstraintStruct
    {
        double max_steer_ = 0.0;
        double max_steer_rate_ = 0.0;
        double max_acceleration_ = 0.0;
        double max_jerk_ = 0.0;
    };

    struct MpcCostWeightStruct
    {
        double w_position_ = 0.0;
        double w_heading_ = 0.0;
        double w_steer_ = 0.0;
        double w_acceleration_ = 0.0;
        double w_jerk_ = 0.0;
    };

    class ConfigReader //配置文件读取器
    {
    public:
        ConfigReader();

        // vehicle
        void read_vehicle_config(VehicleStruct &vehicle, const std::string name);
        inline VehicleStruct main_car() const { return main_car_; }

         // global_path
        void read_global_path_config();
        inline GlobalPathStruct global_path() const { return global_path_; }

        // controller
        void read_controller_type();
        inline ControllerTypeStruct controller_type() const { return controller_type_; }
        
        void read_pid_param();
        inline PidParamStruct pid_param() const { return pid_param_; }

        void read_lqr_dynamic_param();
        inline LqrDynamicParamStruct lqr_dynamic_param() const { return lqr_dynamic_param_; }

        void read_lqr_kinematic_param();
        inline LqrKinematicParamStruct lqr_kinematic_param() const { return lqr_kinematic_param_; }

        void read_mpc_param();
        inline MpcParamStruct mpc_param() const { return mpc_param_; }

        void read_mpc_model();
        inline MpcModelStruct mpc_model() const { return mpc_model_; }

        void read_mpc_hard_constraint();
        inline MpcHardConstraintStruct mpc_hard_constraint() const { return mpc_hard_constraint_; }

        void read_mpc_cost_weight();
        inline MpcCostWeightStruct mpc_cost_weight() const { return mpc_cost_weight_; }

    private:
        YAML::Node controller_config;

        // vehicle
        VehicleStruct main_car_;

        // global_path
        GlobalPathStruct global_path_;

        // controller
        ControllerTypeStruct controller_type_;

        PidParamStruct pid_param_;
        LqrDynamicParamStruct lqr_dynamic_param_;
        LqrKinematicParamStruct lqr_kinematic_param_;
        MpcParamStruct mpc_param_;
        MpcModelStruct mpc_model_;
        MpcHardConstraintStruct mpc_hard_constraint_;
        MpcCostWeightStruct mpc_cost_weight_;

    };
} // namespace Controller

#endif // CONFIG_READER_H_