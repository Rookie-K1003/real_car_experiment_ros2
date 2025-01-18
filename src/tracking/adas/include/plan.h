#ifndef PLAN_H
#define PLAN_H

#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "location_msgs/msg/rtk.hpp"

#include "adas_utils.h"
using namespace std;
class adas_plan : public rclcpp::Node
{
public:
    explicit adas_plan(const rclcpp::NodeOptions & node_options);
    ~adas_plan() {}

    // 读取地图
    void getRouteFromFile();

    // 回调函数
    void recvHuacePosCallback(const location_msgs::msg::RTK::ConstSharedPtr msg);

    // 地图数据
    vector<positionConf> route_data_;
    nav_msgs::msg::Path visu_map_;

    // 目前位置
    positionConf car_;

    // 可视化
    void visulization();

private:
    // 地图原点
    double latitude_;
    double longitude_;
    double height_;

    // Subscriber and Publisher
    rclcpp::Subscription<location_msgs::msg::RTK>::SharedPtr rtk_sub_;
    
    // Visulization publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_pub_;
    
};

#endif