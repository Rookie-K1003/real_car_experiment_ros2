#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/control_req.hpp"
#include "chassis_msgs/msg/chassis_info.hpp"

#include "CanMsgDefs.h"
#include "CanCommonFcn.h"
#include "CanHandler.h"

#include <thread>
#include <mutex>
#include <cmath>

using namespace CanProcess;

class CanCommanderNode : public rclcpp::Node {
public:
    CanCommanderNode() : Node("esdcan") {
        subTrackingControlCmd_ = this->create_subscription<control_msgs::msg::ControlReq>(
            "/tracking_control_req", 1, std::bind(&CanCommanderNode::CallbackTrackingControlCmd, this, std::placeholders::_1));

        subCurrentVelocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/current_velocity", 1, std::bind(&CanCommanderNode::CallbackCurrentVelocity, this, std::placeholders::_1));

        pubChassisInfo_ = this->create_publisher<chassis_msgs::msg::ChassisInfo>("/chassis_info", 1);

        initRecvMessage();
        initSendMessages();
        can_handler_ = std::make_unique<CanHandler>(channel);

        send_ = &sendArg;
        recv_ = &recvArg;
        recv_->channel = channel;
        send_->channel = channel;
        send_->interval = 50;

        if (can_handler_->opened != 0) {
            send_->num_frames = 4;
            send_->data = new CMSG[send_->num_frames];
            for (int i = 0; i < send_->num_frames; ++i) send_->data[i].len = 8;

            std::unique_lock<std::mutex> lock(can_handler_->recv_data_lock);
            g_CurrentAngle = getTurn(g_car);
            setTurn(g_CurrentAngle, g_car);
            send_all_safe(send_);
        }

        setTurn(0, g_car);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(g_dt * 1000)),
            std::bind(&CanCommanderNode::mainLoop, this));
    }

    ~CanCommanderNode() override {
        int numtj = 0;
        while (numtj++ < 10) {
            setTurn(0, g_car);
            send_all_safe(send_);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        numtj = 0;
        while (numtj++ < 10) send_all_safe(send_);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        exitAll(g_car);
        send_all_safe(send_);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        delete[] send_->data;
        send_->run = false;
        recv_->run = false;

        can_handler_->join();
    }

private:
     // === Global state ===
    double g_LastError = 0.0;
    double g_AccumulatedError = 0.0;
    double g_dt = 0.05;
    double g_torque_threshold = 15.0;

    int g_Direction = 1;
    int g_BreakHandBrakeCount = 0;
    int g_KeyBoardInput = 0;
    int g_TurningSystemStatus = 0;

    int g_TargetGear = 1;
    double g_TargetAngle = 0.0;
    double g_TargetSpeed = 0.0;
    double g_TargetAcceleration = 0.0;
    double g_TargetTorque = 0.0;

    int g_CurrentGear = 0;
    double g_CurrentAngle = 0.0;
    double g_CurrentSpeed = 0.0;
    double g_CurrentAcceleration = 0.0;

    int g_LongitudeDrivingStatus = 0;
    int g_HandBrakeStatus = 1;

    int g_CurrentBehaviorState = 0;

    int g_car = 1;  // 1:美团车，0：萌建号
    int channel = 2;

    // === Thread safety ===
    std::mutex g_KeyBoardMutex;
    std::mutex g_CurrentBehaviorMutex;
    std::mutex g_SpeedMutex;

    // ---------- function ----------------
    void CallbackTrackingControlCmd(const control_msgs::msg::ControlReq::SharedPtr msg) {
        int state;
        {
            std::lock_guard<std::mutex> lock(g_CurrentBehaviorMutex);
            state = g_CurrentBehaviorState;
        }
        if (state == 16) return;

        g_TargetSpeed = msg->vel_req;
        g_TargetAngle = msg->angle_req;
    }

    void CallbackCurrentVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        g_CurrentSpeed = msg->twist.linear.x;
    }

    void mainLoop() {
        if (can_handler_->opened == 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN device not opened!");
            return;
        }

        {
            std::unique_lock<std::mutex> lock(can_handler_->recv_data_lock);
            g_Direction = g_TargetGear == 2 ? -1 : 1;
            g_TargetSpeed = std::abs(g_TargetSpeed);
            g_CurrentAngle = getTurn(g_car);
        }

        g_TargetAcceleration = setAccSimplePID(g_CurrentSpeed, g_TargetSpeed, g_dt);
        g_TargetTorque = setTorqueFromAccSimple(g_TargetAcceleration);

        setTurn(g_TargetAngle, g_car);

        if (g_TargetTorque > g_torque_threshold) {
            setBrake(0, 1, g_car);
            setDrive(g_TargetTorque, 1, g_TargetGear, g_car);
        } else if (g_TargetTorque < -g_torque_threshold) {
            setDrive(0, 1, g_TargetGear, g_car);
            setBrake(-g_TargetTorque, 1, g_car);
        } else {
            setDrive(0, 1, g_TargetGear, g_car);
            setBrake(0, 1, g_car);
        }

        if (isADSAvailable()) {
            send_->msg_arrived = true;
            send_all_safe(send_);
        } else {
            RCLCPP_INFO(this->get_logger(), "ADS is not available");
        }

        chassis_msgs::msg::ChassisInfo msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

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

        pubChassisInfo_->publish(msg);
    }

    double setAccSimplePID(double current, double target, double dt) {
        double kp = 2.0, ki = 0.3, kd = 0.1;
        double error = target - current;
        double integral = 0.0, derivative = 0.0;
        double max_acc = 2.0;

        if (error != 0) {
            integral = (error + g_AccumulatedError) * dt;
            derivative = (error - g_LastError) / dt;
        }
        g_LastError = error;
        g_AccumulatedError += error;

        if (std::abs(integral * ki) > max_acc) g_AccumulatedError = 0.0;

        double output = kp * error + ki * integral + kd * derivative;
        return std::clamp(output, -max_acc, max_acc);
    }

    double setTorqueFromAccSimple(double acc) {
        double mass = 590.0, Rr = 0.2;
        double torque = mass * Rr * acc;
        return std::clamp(torque, -200.0, 200.0);
    }

    bool isADSAvailable() {
        return isVCUAvailable() && isEPSAvailable() && isEHBAvailable();
    }

    bool isEPSAvailable() {
        return getEPSDriverSteerStatus() == 0 && getEPSWireControlStatus() == 1;
    }

    bool isEHBAvailable() {
        return getEHBDriverBrakeStatus() == 0 && getEHBWireControlBrakeEnable() == 1;
    }

    bool isVCUAvailable() {
        return getDriverThrottleStatus() == 0 && getWireControlDriveEnable() == 1;
    }

    rclcpp::Subscription<control_msgs::msg::ControlReq>::SharedPtr subTrackingControlCmd_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subCurrentVelocity_;
    rclcpp::Publisher<chassis_msgs::msg::ChassisInfo>::SharedPtr pubChassisInfo_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<CanHandler> can_handler_;
    send_args *send_;
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