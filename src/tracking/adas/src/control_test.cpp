#include <rclcpp/rclcpp.hpp>
#include "control_msgs/msg/control_req.hpp"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <chrono>  // 添加此行

class ControlTestNode : public rclcpp::Node {
public:
    ControlTestNode() : Node("control_test"), control_angle_(0.0), control_speed_(0.0) {
        control_test_pub_ = this->create_publisher<control_msgs::msg::ControlReq>("/tracking_control_req", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControlTestNode::publishControlCommand, this));
    }

private:
    double control_angle_;
    double control_speed_;
    rclcpp::Publisher<control_msgs::msg::ControlReq>::SharedPtr control_test_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishControlCommand() {
        control_msgs::msg::ControlReq control_test_cmd;
        control_test_cmd.vel_req = control_speed_;
        control_test_cmd.angle_req = control_angle_;

        control_test_pub_->publish(control_test_cmd);

        if (kbhit()) {
            char key = getch();
            switch (key) {
                case 'd':  // Increase angle
                    if (control_angle_ < 25.0) {
                        control_angle_ += 1.0;
                    }
                    break;
                case 'a':  // Decrease angle
                    if (control_angle_ > -25.0) {
                        control_angle_ -= 1.0;
                    }
                    break;
                case 'w':  // Increase speed
                    if (control_speed_ < 10.0) {
                        control_speed_ += 1.0;
                    }
                    break;
                case 's':  // Decrease speed
                    if (control_speed_ > 0.0) {
                        control_speed_ -= 1.0;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    char getch() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~ICANON; // Disable canonical mode
        newt.c_lflag &= ~ECHO;   // Disable echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    int kbhit() {
        struct termios oldt, newt;
        int oldf;
        int ch;

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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlTestNode>());
    rclcpp::shutdown();
    return 0;
}