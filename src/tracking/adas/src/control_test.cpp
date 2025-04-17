#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/control_req.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <locale>

class ControlTestNode : public rclcpp::Node {
public:
    ControlTestNode() : Node("control_test"), control_angle_(0.0), control_speed_(0.0),
                        control_steering_angle_(0.0), count_(0), current_speed_(0.0) {
        setlocale(LC_ALL, "");

        // 创建发布器
        control_pub_ = this->create_publisher<control_msgs::msg::ControlReq>("/tracking_control_req", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/current_velocity", 10);

        // 初始化消息
        control_msg_.vel_req = control_speed_;
        control_msg_.angle_req = control_angle_;

        // 注册 Ctrl+C 信号处理器
        signal(SIGINT, signalHandler);
        global_node_ = this;

        // 创建定时器：20Hz 主循环
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&ControlTestNode::mainLoop, this));
    }

private:
    // === 成员变量 ===
    double control_angle_;
    double control_speed_;
    double control_steering_angle_;
    int count_;
    double current_speed_;

    control_msgs::msg::ControlReq control_msg_;
    geometry_msgs::msg::TwistStamped velocity_msg_;

    rclcpp::Publisher<control_msgs::msg::ControlReq>::SharedPtr control_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // === 主循环 ===
    void mainLoop() {
        control_pub_->publish(control_msg_);

        if (kbhit()) {
            char key = getch();
            switch (key) {
                case 'd': if (control_angle_ < 25.0) control_angle_ += 1.0; break;
                case 'a': if (control_angle_ > -25.0) control_angle_ -= 1.0; break;
                case 'w': if (control_speed_ < 10.0) control_speed_ += 1.0; break;
                case 's': if (control_speed_ > 0.0) control_speed_ -= 1.0; break;
                default: break;
            }

            control_steering_angle_ = control_angle_ * 13.7;
            control_msg_.vel_req = control_speed_;
            control_msg_.angle_req = control_steering_angle_;

            RCLCPP_INFO(this->get_logger(), "SteeringAngle = %.2f, Speed = %.2f",
                        control_steering_angle_, control_speed_);
        }

        // 模拟当前速度变化
        if (count_ <= 400) {
            current_speed_ = 0.0;
            count_++;
        } else if (count_ <= 800) {
            current_speed_ = 15.0;
            count_++;
        } else {
            current_speed_ = 0.0;
            count_ = 0;
        }

        velocity_msg_.twist.linear.x = current_speed_;
        velocity_pub_->publish(velocity_msg_);
    }

    // === 工具函数 ===
    char getch() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~ICANON;
        newt.c_lflag &= ~ECHO;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    int kbhit() {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }

    // === 静态信号处理器 ===
    static void signalHandler(int signum) {
        if (global_node_) {
            RCLCPP_INFO(global_node_->get_logger(), "Interrupt signal (%d) received. Shutting down...", signum);
        }
        rclcpp::shutdown();
    }

    // === 全局指针供 signal handler 使用 ===
    static inline ControlTestNode* global_node_ = nullptr;
};

// === 主函数入口 ===
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}