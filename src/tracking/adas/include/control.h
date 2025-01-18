#ifndef CONTROL_H
#define CONTROL_H

#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "control_msgs/msg/control_req.hpp"
#include "location_msgs/msg/rtk.hpp"

#include "adas_utils.h"

using namespace std;
class adas_control : public rclcpp::Node
{
public:
    explicit adas_control(const rclcpp::NodeOptions & node_options);
    ~adas_control(){};
    // 主函数
    void cal_control(const positionConf &car_, const vector<positionConf> &route_data_);
    void send_control();

    // 在点集中找最近点，返回在vector中的index，可提供先验
    void findClosestPoint(const vector<positionConf> &data, int &prior_index, const positionConf &p);
    // 两个点的距离
    double pointDistanceSquare(const positionConf &p1, const positionConf &p2);
    // 车辆中心->前轴中心
    void center2front(const positionConf &center, positionConf &front);

    int findRealCloseIndex(const int &close_index_theory, const positionConf &p, const vector<positionConf> &route_data_);

    // 找计算曲率半径的点
    void findPoints2ComputeRadius(const int &close_index, int &index_r1,
                                           int &index_f1, int &index_f2, int &index_f3,
                                           const vector<positionConf> &route_data_);

    // 计算曲率半径
    double getR(const positionConf &pt1, const positionConf &pt2, const positionConf &pt3);

    // 找预瞄点
    void findPrePoint(positionConf &pre_p, const int &index, const double &pre_length,
                      const vector<positionConf> &route_data_);

    // 地图原点的ENU坐标系->当前的车辆坐标系
    void enu2veh(positionConf &p_veh, const positionConf &p, const positionConf &xy_pos);

    // 根据横向控制模型计算横向误差
    double StanleyPreviewLatteralModel(const positionConf &xy_p, const positionConf &vnxy_p,
                                       const positionConf &vnxy_p_fc, const positionConf &vpxy_p);

    // heading2 -> heading1 旋转的角度，-180 ~ 180 顺时针为正
    double computedheading(const double &heading1, const double &heading2);

    // 可视化
    void visulization(const positionConf &xy_pos, const positionConf &close_pos, const positionConf &pre_pos);

    //栅格地图回调函数
    void callbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

    //判断停障
    bool checkObstacle(const positionConf &car_, const vector<positionConf> &route_data_);

    double control_speed_;
private:
    // 系统参数
    //  惯导安装位置
    double insgps_x_;
    double insgps_y_;
    // 车辆中心-前轴中心距离
    double center2frontaxis_;
    // 系统延迟时间
    double sys_delay_;
    // 最大曲率半径
    double Rmax_;
    //  传动转向比
    double turn_ratio_;
    // 方向盘最大转角
    double max_turn_angle_;
    // 最大速度
    double max_speed_;

    // 预瞄参数
    int turnleft_ispositive_;
    // 预瞄时间
    double preview_time_;
    // 最小最大基础预瞄距离
    double basic_prelength_min_;
    double basic_prelength_max_;

    // 预瞄模型比例
    double lamda_preview_;
    double lamda_stanley_;
    double k_stanley_;
    double k_soft_;

    // 地图原点
    double latitude_;
    double longitude_;
    double height_;

    // 最近点
    int close_index_;
    int finish_index_;

    // PID 参数
    
    std::shared_ptr<PIDControl> pid_control_angle_;
    std::shared_ptr<PIDControl> pid_control_speed_;
    PIDConf pid_conf_angle_;
    PIDConf pid_conf_speed_;

    double control_angle_;
    double previous_control_angle_;

    //栅格地图数据
    vector<int> GridMapData;
    //栅格地图原点到车辆坐标系原点的距离，等于0.5倍的栅格地图长宽
    double half_width;
    double half_length;
    double resolution;

    // Subscriber and Publisher
    rclcpp::Publisher<control_msgs::msg::ControlReq>::SharedPtr control_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

    // Visulization publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Pose_car_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Pose_close_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Pose_pre_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    //for dwa
    geometry_msgs::msg::PoseStamped goal_pose_;
};

#endif