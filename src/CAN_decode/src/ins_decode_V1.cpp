/*
    修改原组合导航CAN解析代码，适配ros2自带的位姿、imu、里程计等msg类型
    @author: KQ
    @date: 2025-04-17
    @note: 
    1. 发布ros2自带的位姿、imu、里程计等msg类型
    2. 将原有rtk使用的原始角速度和加速度改为车辆坐标系加速度和角速度
    3. 增加lla转xyz，配置文件传入自定义map起点的lla坐标，转换出map坐标系下相对的xyz
*/
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "location_msgs/msg/rtk.hpp"
#include <ntcan.h>
#include "lla2map_converter.hpp"

using namespace std;

#define NEWFLAG -100000
NTCAN_HANDLE dev_handler_;

class can_decode_class : public rclcpp::Node
{
public:
    explicit can_decode_class(const rclcpp::NodeOptions &node_options)
        : Node("can_decode", node_options), converter(latitude_origin_, longitude_origin_, altitude_origin_)
    {
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("current_odom", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("current_imu", 10);
        rtk_pub_ = create_publisher<location_msgs::msg::RTK>("rtk_data", 10);
        velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("current_velocity", 10);

        // 读取配置文件
        // 读取配置文件中的参数
        this->declare_parameter("localization.latitude_origin", 0.0);
        this->declare_parameter("localization.longitude_origin", 0.0);
        this->declare_parameter("localization.altitude_origin", 0.0);

        this->get_parameter("localization.latitude_origin", latitude_origin_);
        this->get_parameter("localization.longitude_origin", longitude_origin_);
        this->get_parameter("localization.altitude_origin", altitude_origin_);

}


    }

    ~can_decode_class() {}

    // 节点主函数
    void main_process()
    {
        int channel = 3; // CAN通道号
        int32_t ret = canOpen(channel, 0, NTCAN_MAX_TX_QUEUESIZE, NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);
        if (ret != NTCAN_SUCCESS) {
            std::cout << "Open device error code: " << ret << std::endl;
            return;
        }
        else
        {
            std::cout << "open device sussess:\n";
        }

        // 1. set receive message_id filter, ie white list 大概是有个CANid的滤波器，要把要的发过去
        int32_t id_count = 0x800;
        // @JackFine 此处的函数具体实现，是否为库函数
        ret = canIdRegionAdd(dev_handler_, 0, &id_count); // 相当于从0到0+0x800都会被通过

        if (ret != NTCAN_SUCCESS)
        {
            std::cout << "add receive msg id filter error\n";
        }
        else
        {
            std::cout << "add receive msg id filter success\n";
        }

        // 2. set baudrate to 500k 波特率设置
        // @JackFine 此处设置波特率
        ret = canSetBaudrate(dev_handler_, NTCAN_BAUD_500);

        if (ret != NTCAN_SUCCESS)
        {
            std::cout << "set baudrate error\n";
        }
        else
        {
            std::cout << "set baudrate success\n";
        }
        
        // CAN消息接收数组和频率
        CMSG recv[1000];
        rclcpp::Rate r(100); // 组合导航频率100Hz

        // 要发布的ROS消息
        geometry_msgs::msg::PoseStamped pose_data;
        nav_msgs::msg::Odometry odom_data;
        sensor_msgs::msg::Imu imu_data;
        location_msgs::msg::RTK rtk_data;
        geometry_msgs::msg::TwistStamped velocity_data;

        // 节点循环接收CAN消息并解析
        while (rclcpp::ok()) {
            int32_t frame_num = 100;
            const int ret2 = canTake(dev_handler_, recv, &frame_num);
            if (ret2 == NTCAN_SUCCESS && frame_num != 0) {
                // 解析不同CAN ID的消息
                for (int i = 0; i < frame_num; ++i) {
                    process_can_message(recv[i]);
                }
            }

            // 将lla转map坐标系下的x,y,z
            auto [x, y, z] = converter.convert(latitude_, longitude_, altitude_);
            map_x_ = x;
            map_y_ = y;
            map_z_ = z;

            // 发布ROS消息
            publish_pose(pose_data); // geometry_msgs/msg/pose_stamped
            publish_odom(odom_data); // nav_msgs/msg/odometry
            publish_imu(imu_data);
            publish_rtk(rtk_data);
            publish_velocity(velocity_data); // geometry_msgs/msg/twist_stamped

            r.sleep();
        }

        // @JackFine CAN解析结束
        int32_t ret3 = canClose(dev_handler_);
        if (ret3 != NTCAN_SUCCESS)
        {
            std::cout << "close error code:\n";
        }
        else
        {
            std::cout << "close esd can ok. port\n";
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<location_msgs::msg::RTK>::SharedPtr rtk_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    
    LlaToMapConverter converter;  // 类成员变量
    // 其他的一些解析变量
    int system_state_; // 系统状态 1-卫导，2-组合导航模式，3-纯惯导模式
    int satellite_status_; // 卫星状态 4和5为RTK模式
    double latitude_; // 纬度
    double longitude_; // 经度
    double altitude_; // 高度
    double velocity_; // 车辆速度
    double acc_x_; // 车辆坐标系加速度x轴(单位g)
    double acc_y_; // 车辆坐标系加速度y轴(单位g)
    double acc_z_; // 车辆坐标系加速度z轴(单位g)
    double roll_; // 横滚角(单位deg)
    double pitch_; // 俯仰角(单位deg)
    double yaw_; // 偏航角(单位deg)
    double angrate_x_; // 车辆坐标系角速度X轴(单位rad/s)
    double angrate_y_; // 车辆坐标系角速度Y轴(单位rad/s)
    double angrate_z_; // 车辆坐标系角速度Z轴(单位rad/s)
    double map_x_; // 地图坐标系下X坐标
    double map_y_; // 地图坐标系下Y坐标
    double map_z_; // 地图坐标系下Z坐标
    double latitude_origin_ = 31.2304; // 地图坐标系原点纬度
    double longitude_origin_ = 121.4737; // 地图坐标系原点经度
    double altitude_origin_ = 0.0; // 地图坐标系原点高度

    // 计算、数据转换用到的常量
    const double e = 0.0818191908425;
    const double R = 6378137;
    const double torad = M_PI / 180;
    const double g = 9.7964;
    double Re = 0;

    int timestamp_type_; // 时间戳类型：0-UTC时间，1-GPS时间，2-ROS时间

    /**
     * @brief 处理CAN消息
     *
     * 根据CAN消息的ID，解析不同的数据并赋值给成员变量
     *
     * @param msg CAN消息结构体
     */
    void process_can_message(CMSG& msg) {
        if (msg.id == 803) {
            // INS状态
            // 一些系统定位的,暂时先不处理
            rtk_data.status = CAN_decode(recv[i], 16, 8, 1, 0, 0); // RTK稳定解定位定向时，正常应当为status4
        } else if (msg.id == 804) {
            // GPS经纬度
            latitude_ = CAN_decode(msg, 0, 32, 1e-7, 0, 0);
            longitude_ = CAN_decode(msg, 32, 32, 1e-7, 0, 0);
            rtk_data.latitude = latitude_;
            rtk_data.longitude = longitude_;            
        } else if (msg.id == 801) {
            // IMU角速度原始值
            rtk_data.angrate_raw_x = CAN_decode(recv[i], 0, 20, 1e-2, 0, 1) * torad;
            rtk_data.angrate_raw_y = CAN_decode(recv[i], 20, 20, 1e-2, 0, 1) * torad;
            rtk_data.angrate_raw_z = CAN_decode(recv[i], 40, 20, 1e-2, 0, 1) * torad;

        } else if (msg.id == 802) {
            // IMU加速度原始值
            // 处理消息并发布到相应的话题
            rtk_data.accel_raw_x = CAN_decode(recv[i], 0, 20, 1e-4, 0, 1) * g;
            rtk_data.accel_raw_y = CAN_decode(recv[i], 20, 20, 1e-4, 0, 1) * g;
            rtk_data.accel_raw_z = CAN_decode(recv[i], 40, 20, 1e-4, 0, 1) * g;           

        } else if (msg.id == 805) {
            // 位置高度信息
            altitude_ = CAN_decode(msg, 0, 32, 1e-3, 0, 0);
            rtk_data.height = altitude_;
        } else if (msg.id == 806) {
            // 位置sigma值（定位误差？）

        } else if (msg.id == 807) {
            // 车辆速度
            velocity_ = CAN_decode(msg, 48, 16, 1e-2, 0, 0); // 需要验证和轮速误差得到车速的误差
            rtk_data.velocity = velocity_;
        }
        else if (msg.id == 808) {
            // 速度sigma
        }
        else if (msg.id == 809) {
            // 车辆坐标系加速度（单位转换为m/s^2）
            acc_x_ = CAN_decode(msg, 0, 20, 1e-4, 0, 1) * g;
            acc_y_ = CAN_decode(msg, 20, 20, 1e-4, 0, 1) * g;
            acc_z_ = CAN_decode(msg, 40, 20, 1e-4, 0, 1) * g;
        }
        else if (msg.id == 810) {
            // 姿态角 RPY(单位先用deg)
            roll_ = CAN_decode(msg, 32, 16, 1e-2, 0, 1);
            pitch_ = CAN_decode(msg, 16, 16, 1e-2, 0, 1);
            yaw_ = CAN_decode(msg, 0, 16, 1e-2, 0, 0);
            rtk_data.heading = yaw_;
            rtk_data.pitch = pitch_;
            rtk_data.roll = roll_;
        }
        else if (msg.id == 811) {
            // 姿态角sigma
        }
        else if (msg.id == 812) {
            // 车辆坐标系角速度(单位转换成rad/s)
            angrate_x_ = CAN_decode(msg, 0, 20, 1e-2, 0, 1) * torad;
            angrate_y_ = CAN_decode(msg, 20, 20, 1e-2, 0, 1) * torad;
            angrate_z_ = CAN_decode(msg, 40, 20, 1e-2, 0, 1) * torad;
        }
        else {}
    }

    /**
     * @brief 从CAN消息中解码出相应的数据
     *
     * 从给定的CAN消息中提取特定长度的数据，并根据比例和偏移量进行转换。
     *
     * @param raw_data 原始的CAN消息数据
     * @param lsb 最低有效位的起始位置（从0开始计数）
     * @param length 要解码的数据长度（以位为单位）
     * @param ratio 转换比例
     * @param bias 偏移量
     * @param mode 解码模式（默认为0，1表示使用补码转换）
     *
     * @return 解码后的浮点数结果
     */
    double CAN_decode(CMSG raw_data, int lsb, int length, double ratio, double bias, int mode = 0) {
        // 实现相应的解码逻辑
        // ... (保持原有解码逻辑)
        // @JackFine 需要重新写
        // 首先确定lsb的位置属于哪个字节
        int lsb_byte = lsb / 8;
        // 然后确定lsb所在的位
        int lsb_bit = lsb % 8;
        // 再确定msb所在的位
        int msb_bit = (length + lsb_bit - 1) % 8;

        // 求总字节数量
        int num_byte;
        if ((lsb_bit + length - 1) / 8 == 0)
            num_byte = 1;
        else
            num_byte = 1 + 1 + (length - (8 - lsb_bit) - (msb_bit + 1)) / 8;
        int msb_byte = lsb_byte + num_byte - 1; // 求高8位起始字节位置
        
        int deviation = 0;
        int data = 0;

        if (num_byte == 1)
        {
            int tmp = 0;
            for (int i = lsb_bit; i <= msb_bit; i++)
                tmp += (int)pow(2, i);
            data += (int)(raw_data.data[lsb_byte] & tmp); // @JackFine 按位与，提取单个字符串中低位到高位中的所有信息
            data = ratio * data + bias;
            return data;
        }

        for (int byte = lsb_byte; byte < lsb_byte + num_byte; byte++) // @JackFine 从低到高不断拼接can帧
        {
            // printf("calculate byte %d : %lf \n", byte, data);
            if (byte == lsb_byte) // @JackFine 低位开始字节
            {
                int tmp = 0;
                for (int i = lsb_bit; i <= 7; i++)
                    tmp += round(pow(2, i));
                // std::cout << "the bit of first = " << bitset<sizeof((raw_data.data[byte] & tmp)) * 8>((raw_data.data[byte] & tmp)) << std::endl;
                data += (int)((raw_data.data[byte] & tmp) >> lsb_bit); // @JackFine 提取有效字段
                deviation += (8 - lsb_bit);
            }
            else if (byte == lsb_byte + num_byte - 1) // @JackFine 最后一个字节
            {
                int tmp = 0;
                for (int i = 0; i <= msb_bit; i++)
                    tmp += round(pow(2, i));
                // std::cout << "the bit of tmp = " << bitset<sizeof((raw_data.data[byte] & tmp)) * 8>((raw_data.data[byte] & tmp)) << std::endl;
                data += (int)((raw_data.data[byte] & tmp) << deviation); // @JackFine 提取有效字段
            }
            else
            {
                // std::cout << "the bit of data = " << bitset<sizeof((raw_data.data[byte])) * 8>(raw_data.data[byte]) << std::endl;
                data += (int)((raw_data.data[byte]) << deviation); // @JackFine 读取所有的数据
                deviation += 8;
            }
        }

        // 如果最高位是符号位，并且为1，则应用补码转换，将解码后的值转换为负数
        if (mode == 1)
        {
            int tmp = round(pow(2, msb_bit));
            if ((raw_data.data[msb_byte] & tmp) == tmp)
            {
                data = (-1) * (pow(2, length) - data - 1);
            }
        }

        double output = ratio * data + bias;
        return output;
    }

    void publish_pose(geometry_msgs::msg::PoseStamped pose)
    {
        // 时间戳、坐标系
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";

        // 位置
        pose.pose.position.x = map_x_;
        pose.pose.position.y = map_y_;
        pose.pose.position.z = map_z_;
        // 姿态
        // rpy转四元数,注意单位要用弧度制
        pose.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(roll_ * torad, pitch_ * torad, yaw_ * torad);
        
        // 发布话题
        pose_pub_->publish(pose);
    }
    void publish_odom(nav_msgs::msg::Odometry odom)
    {
        // 时间戳、坐标系
        odom.header.stamp = rclcpp::Clock().now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";

        // 位置
        odom.pose.pose.position.x = map_x_;
        odom.pose.pose.position.y = map_y_;
        odom.pose.pose.position.z = map_z_;
        // 姿态
        // rpy转四元数,注意单位要用弧度制
        odom.pose.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(roll_ * torad, pitch_ * torad, yaw_ * torad);
        // 速度
        odom.twist.twist.linear.x = velocity_;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = angrate_x_;
        odom.twist.twist.angular.y = angrate_y_;
        odom.twist.twist.angular.z = angrate_z_;

        // 发布话题
        odom_pub_->publish(odom);
    }
    void publish_imu(sensor_msgs::msg::Imu imu)
    {
        // 时间戳、坐标系
        imu.header.stamp = rclcpp::Clock().now();
        imu.header.frame_id = "base_link";

        // 姿态
        // rpy转四元数,注意单位要用弧度制
        imu.orientation = tf2::createQuaternionMsgFromRollPitchYaw(roll_ * torad, pitch_ * torad, yaw_ * torad);
        // 角速度
        imu.angular_velocity.x = angrate_x_;
        imu.angular_velocity.y = angrate_y_;
        imu.angular_velocity.z = angrate_z_;
        // 线加速度
        imu.linear_acceleration.x = velocity_;
        imu.linear_acceleration.y = 0;
        imu.linear_acceleration.z = 0;

        // 发布话题
        imu_pub_->publish(imu);
    }
    void publish_rtk(location_msgs::msg::RTK rtk)
    {
        // 时间戳、坐标系
        rtk.header.stamp = rclcpp::Clock().now();
        rtk.header.frame_id = "map";
        // 解析CAN报文时就赋值了，直接发布
        rtk_pub_->publish(rtk);
    }
    void publish_velocity(geometry_msgs::msg::Twist twist)
    {
        // 时间戳、坐标系
        twist.header.stamp = rclcpp::Clock().now();
        twist.header.frame_id = "base_link";
        // 速度
        twist.linear.x = velocity_;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = angrate_x_;
        twist.angular.y = angrate_y_;
        twist.angular.z = angrate_z_;
        // 发布话题
        velocity_pub_->publish(twist);
    }

};

// 节点主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto can_decoder = std::make_shared<can_decode_class>(rclcpp::NodeOptions{});
    can_decoder->main_process();
}