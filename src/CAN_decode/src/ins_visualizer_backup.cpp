#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "location_msgs/msg/rtk.hpp"
#include <nav_msgs/msg/path.hpp>

using namespace std;

class VehicleVisualizer : public rclcpp::Node {
public:
    VehicleVisualizer() : Node("vehicle_visualizer") {
        // 订阅自车定位相关话题
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10, std::bind(&VehicleVisualizer::pose_callback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "current_odom", 10, std::bind(&VehicleVisualizer::odom_callback, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "current_imu", 10, std::bind(&VehicleVisualizer::imu_callback, this, std::placeholders::_1));
        velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "current_velocity", 10, std::bind(&VehicleVisualizer::velocity_callback, this, std::placeholders::_1));
        rtk_sub_ = create_subscription<location_msgs::msg::RTK>(
            "rtk_data", 10, std::bind(&VehicleVisualizer::rtk_callback, this, std::placeholders::_1));

        // 发布可视化的消息（带_rviz后缀）
        pose_rviz_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_rviz", 10);
        odom_rviz_pub_ = create_publisher<nav_msgs::msg::Odometry>("current_odom_rviz", 10);
        imu_rviz_pub_ = create_publisher<sensor_msgs::msg::Imu>("current_imu_rviz", 10);
        velocity_rviz_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("current_velocity_rviz", 10);
        rtk_rviz_pub_ = create_publisher<location_msgs::msg::RTK>("rtk_data_rviz", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("vehicle_marker_rviz", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("vehicle_path_rviz", 10);

        // 读取配置参数（车辆尺寸）
        this->declare_parameter("vehicle.length", 4.0);   
        this->declare_parameter("vehicle.width", 2.0);    
        this->declare_parameter("vehicle.height", 1.5);   
        this->get_parameter("vehicle.length", length_);
        this->get_parameter("vehicle.width", width_);
        this->get_parameter("vehicle.height", height_);

        // 初始化路径
        path_.header.frame_id = "map";
    }

private:
    // 订阅回调函数
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 发布带 _rviz 后缀的可视化话题
        geometry_msgs::msg::PoseStamped pose_rviz = *msg;
        pose_rviz_pub_->publish(pose_rviz);

        // 更新bounding box
        update_bounding_box(msg);

        // 更新轨迹
        update_trajectory(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 发布带 _rviz 后缀的可视化话题
        nav_msgs::msg::Odometry odom_rviz = *msg;
        odom_rviz_pub_->publish(odom_rviz);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 发布带 _rviz 后缀的可视化话题
        sensor_msgs::msg::Imu imu_rviz = *msg;
        imu_rviz_pub_->publish(imu_rviz);
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // 发布带 _rviz 后缀的可视化话题
        geometry_msgs::msg::TwistStamped velocity_rviz = *msg;
        velocity_rviz_pub_->publish(velocity_rviz);
    }

    void rtk_callback(const location_msgs::msg::RTK::SharedPtr msg) {
        // 发布带 _rviz 后缀的可视化话题
        location_msgs::msg::RTK rtk_rviz = *msg;
        rtk_rviz_pub_->publish(rtk_rviz);
        // 更新经纬度和RTK状态信息
        update_rtk_info(msg);
    }

    // 绘制bounding box
    void update_bounding_box(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        visualization_msgs::msg::Marker bounding_box;
        bounding_box.header.frame_id = "map";
        bounding_box.header.stamp = this->get_clock()->now();
        bounding_box.ns = "vehicle";
        bounding_box.id = 0;
        bounding_box.type = visualization_msgs::msg::Marker::CUBE;
        bounding_box.action = visualization_msgs::msg::Marker::ADD;
        bounding_box.pose = msg->pose;
        bounding_box.scale.x = length_;
        bounding_box.scale.y = width_;
        bounding_box.scale.z = height_;
        bounding_box.color.r = 0.0;
        bounding_box.color.g = 1.0;
        bounding_box.color.b = 0.0;
        bounding_box.color.a = 0.5;
        marker_pub_->publish(bounding_box);
    }

    // 更新显示RTK的经纬度和状态信息
    void update_rtk_info(const location_msgs::msg::RTK::SharedPtr msg) {
        visualization_msgs::msg::Marker rtk_marker;
        rtk_marker.header.frame_id = "map";
        rtk_marker.header.stamp = this->get_clock()->now();
        rtk_marker.ns = "rtk_status";
        rtk_marker.id = 1;
        rtk_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        rtk_marker.pose.position.x = map_x_;
        rtk_marker.pose.position.y = map_y_;
        rtk_marker.pose.position.z = map_z_;
        rtk_marker.scale.z = 0.5;
        rtk_marker.color.a = 1.0;
        rtk_marker.color.b = 1.0;
        rtk_marker.text = "Lat: " + to_string(msg->latitude) + " Lon: " + to_string(msg->longitude) + 
                          " RTK Status: " + to_string(msg->status);
        marker_pub_->publish(rtk_marker);
    }

    // 更新轨迹
    void update_trajectory(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 将新的位姿点添加到轨迹中
        path_.poses.push_back(*msg);
        
        // 发布轨迹
        path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(path_);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<location_msgs::msg::RTK>::SharedPtr rtk_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rviz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_rviz_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_rviz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_rviz_pub_;
    rclcpp::Publisher<location_msgs::msg::RTK>::SharedPtr rtk_rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    double length_, width_, height_;  // 车辆长宽高
    double map_x_, map_y_, map_z_;    // 地图坐标
    nav_msgs::msg::Path path_;         // 用于存储轨迹
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto vehicle_visualizer = std::make_shared<VehicleVisualizer>();
    rclcpp::spin(vehicle_visualizer);
    rclcpp::shutdown();
    return 0;
}