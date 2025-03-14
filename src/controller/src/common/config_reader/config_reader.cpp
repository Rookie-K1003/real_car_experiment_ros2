/*
文件名：配置文件读取器
作者：kq
日期：2025.03.11
*/

#include "config_reader.h"

namespace Controller
{
    ConfigReader::ConfigReader() // 配置文件读取器
    {
        // 获取workspace/install/controller/share/controller/目录路径
        std::string controller_share_directory = ament_index_cpp::get_package_share_directory("controller");

        // 然后获取配置文件
        controller_config = YAML::LoadFile(controller_share_directory + "/config/controller_config_V1.yaml");
    }
    void ConfigReader::read_vehicle_config(VehicleStruct &vehicle, const std::string name)
    {
        vehicle.id_ = controller_config["vehicle"][name]["id"].as<int>();
        vehicle.frame_ = controller_config["vehicle"][name]["frame"].as<std::string>();
        vehicle.length_ = controller_config["vehicle"][name]["length"].as<double>();
        vehicle.width_ = controller_config["vehicle"][name]["width"].as<double>();
        vehicle.pose_x_ = controller_config["vehicle"][name]["pose_x"].as<double>();
        vehicle.pose_y_ = controller_config["vehicle"][name]["pose_y"].as<double>();
        vehicle.pose_theta_ = controller_config["vehicle"][name]["pose_theta"].as<double>();
        vehicle.speed_ori_ = controller_config["vehicle"][name]["speed_ori"].as<double>();
    }

    void ConfigReader::read_global_path_config()
    {
        try
        {
            global_path_.type_ = controller_config["global_path"]["type"].as<int>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load global_path config: %s", e.what());
        }
    }
    void ConfigReader::read_controller_type()
    {
        try
        {
            controller_type_.type_ = controller_config["controller"]["type"].as<int>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load controller_type config: %s", e.what());
        }
    }
    
    // ToDo:几个基本控制器的参数读取
    void ConfigReader::read_pid_param()
    {
        
    }
    void ConfigReader::read_lqr_dynamic_param()
    {
    }
    void ConfigReader::read_lqr_kinematic_param()
    {
    }
    
    void ConfigReader::read_mpc_param()
    {
        try
        {
            mpc_param_.dt_ = controller_config["mpc_controller"]["dt"].as<double>();
            mpc_param_.pred_horizon_ = controller_config["mpc_controller"]["pred_horizon"].as<int>();
            mpc_param_.control_horizon_ = controller_config["mpc_controller"]["control_horizon"].as<int>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load mpc_param config: %s", e.what());
        }
    }
    void ConfigReader::read_mpc_model()
    {
        try
        {
            mpc_model_.lf_ = controller_config["controller_vehicle_model"]["lf"].as<double>();
            mpc_model_.lr_ = controller_config["controller_vehicle_model"]["lr"].as<double>();
            mpc_model_.m_ = controller_config["controller_vehicle_model"]["m"].as<double>();
            mpc_model_.Iz_ = controller_config["controller_vehicle_model"]["Iz"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load mpc_model config: %s", e.what());
        }
    }
    void ConfigReader::read_mpc_hard_constraint()
    {
        try
        {
            mpc_hard_constraint_.max_steer_ = controller_config["mpc_controller"]["max_steer"].as<double>();
            mpc_hard_constraint_.max_steer_rate_ = controller_config["mpc_controller"]["max_steer_rate"].as<double>();
            mpc_hard_constraint_.max_acceleration_ = controller_config["mpc_controller"]["max_acceleration"].as<double>();
            mpc_hard_constraint_.max_jerk_ = controller_config["mpc_controller"]["max_jerk"].as<double>();
            mpc_hard_constraint_.min_acceleration_ = controller_config["mpc_controller"]["min_acceleration"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load mpc_hard_constraint config: %s", e.what());
        }
    }
    void ConfigReader::read_mpc_cost_weight()
    {
        try
        {
            mpc_cost_weight_.w_position_ = controller_config["mpc_controller"]["w_position"].as<double>();
            mpc_cost_weight_.w_heading_ = controller_config["mpc_controller"]["w_heading"].as<double>();
            mpc_cost_weight_.w_steer_ = controller_config["mpc_controller"]["w_steer"].as<double>();
            mpc_cost_weight_.w_acceleration_ = controller_config["mpc_controller"]["w_acceleration"].as<double>();
            mpc_cost_weight_.w_jerk_ = controller_config["mpc_controller"]["w_jerk"].as<double>();
            mpc_cost_weight_.w_velocity_ = controller_config["mpc_controller"]["w_velocity"].as<double>();
            mpc_cost_weight_.w_steer_rate_ = controller_config["mpc_controller"]["w_steer_rate"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load mpc_cost_weight config: %s", e.what());
        }
    }
} // namespace Controller