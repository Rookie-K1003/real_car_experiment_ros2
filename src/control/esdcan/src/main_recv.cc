#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/control_req.hpp"
#include "chassis_msgs/msg/chassis_info.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "CanMsgDefs.h"
#include "CanCommonFcn.h"
#include "CanHandler.h"

#include <thread>
#include <mutex>
#include <cmath>

using namespace CanProcess;
using namespace std;

class CanCommanderNode : public rclcpp::Node {
public:
    CanCommanderNode() : Node("esdcan") {

        pubChassisInfo_ = this->create_publisher<chassis_msgs::msg::ChassisInfo>("chassis_info", 1);
        // 创建可视化发布器
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("chassis_info_marker_rviz", 10);

        initRecvMessage();
        can_handler_ = std::make_unique<CanHandler>(channel);

        recv_ = &recvArg;
        recv_->channel = channel;

        if (can_handler_->opened != 0) {
            std::unique_lock<std::mutex> lock(can_handler_->recv_data_lock);
            g_CurrentAngle = getTurn(g_car);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(g_dt * 1000)),
            std::bind(&CanCommanderNode::mainLoop, this));
    }

    ~CanCommanderNode() override {

        std::this_thread::sleep_for(std::chrono::seconds(1));

        exitAll(g_car);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        recv_->run = false;

        can_handler_->join();
    }

private:
     // === Global state ===
    int g_car = 1;  // 1:美团车，0：萌建号
    int channel = 2;
    double g_CurrentAngle;
    double g_dt = 0.05;

    // === Thread safety ===
    std::mutex g_KeyBoardMutex;
    std::mutex g_CurrentBehaviorMutex;
    std::mutex g_SpeedMutex;

    // ---------- function ----------------

    void mainLoop() {
        if (can_handler_->opened == 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN device not opened!");
            return;
        }

        {
            std::unique_lock<std::mutex> lock(can_handler_->recv_data_lock);
            g_CurrentAngle = getTurn(g_car);
        }

        // 组织底盘CAN消息并发布
        chassis_msgs::msg::ChassisInfo msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.eps.driver_steer_status = getEPSDriverSteerStatus();
        msg.eps.wire_control_drive_enable = getEPSWireControlStatus();
        msg.eps.steering_angle = getTurn(g_car);

        msg.ehb.driver_brake_status = getEHBDriverBrakeStatus();
        msg.ehb.wire_control_brake_enable = getEHBWireControlBrakeEnable();
        msg.ehb.brk_trq_act = getEHBBrakeTorque();

        msg.vcu.driver_throttle_status = getDriverThrottleStatus();
        msg.vcu.wire_control_drive_enable = getWireControlDriveEnable();
        msg.vcu.actual_gear_shift_position = getActualGearShiftPosition();
        msg.vcu.motor_torque = getMotorPresentTorque();

        msg.esc.esc_wheel_speed_avg = getWheelSpeed(g_car);
        msg.esc.esc_vehicle_speed = getESCVelocity(g_car);

        pubChassisInfo_->publish(msg);

        // 可视化
        visualize_chassis_info(msg);
    }

    // 可视化函数，将ChassisInfo显示为TextMarker
    void visualize_chassis_info(const chassis_msgs::msg::ChassisInfo& msg) {
        visualization_msgs::msg::Marker chassis_info_marker;
        chassis_info_marker.header.frame_id = "map";  // 使用map作为参考坐标系
        chassis_info_marker.header.stamp = this->get_clock()->now();
        chassis_info_marker.ns = "chassis_info";
        chassis_info_marker.id = 0;
        chassis_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        // 设置文本显示位置：左上角
        chassis_info_marker.pose.position.x = 6.0;
        chassis_info_marker.pose.position.y = -5.0;  // 偏右一些
        chassis_info_marker.pose.position.z = 1.0;
        chassis_info_marker.scale.z = 0.5;
        chassis_info_marker.color.a = 1.0;
        chassis_info_marker.color.r = 1.0;  // 白色
        chassis_info_marker.color.g = 1.0;
        chassis_info_marker.color.b = 1.0;

        // 格式化显示内容
        chassis_info_marker.text = 
            "Steering Angle: " + to_string(msg.eps.steering_angle) + " deg\n" +
            "Brake Torque: " + to_string(msg.ehb.brk_trq_act) + " Nm\n" +
            "Drive Torque: " + to_string(msg.vcu.motor_torque) + " Nm\n" +
            "Wheel Speed: " + to_string(msg.esc.esc_wheel_speed_avg) + " m/s\n" +
            "Vehicle Speed: " + to_string(msg.esc.esc_vehicle_speed) + " m/s";

        // 发布显示
        marker_pub_->publish(chassis_info_marker);
    }


    rclcpp::Publisher<chassis_msgs::msg::ChassisInfo>::SharedPtr pubChassisInfo_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // 用于可视化发布器
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<CanHandler> can_handler_;
    recv_args *recv_;
};

// main 函数入口
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanCommanderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}