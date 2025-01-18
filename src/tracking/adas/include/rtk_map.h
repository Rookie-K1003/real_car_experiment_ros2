#ifndef RTK_MAP_H
#define RTK_MAP_H
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "adas_utils.h"
#include "location_msgs/msg/rtk.hpp"

using namespace std;

class rtk_map : public rclcpp::Node
{
public:
    explicit rtk_map(const rclcpp::NodeOptions & node_options);
    ~rtk_map() {}

    // 回调函数
    void recvHuacePosCallback(const location_msgs::msg::RTK::ConstSharedPtr msg);

    // 保存地图函数
    void save2File();

    // 主循环
    void msgs2points();

    // 键盘操作函数
    int get_char();

private:
    //  惯导安装位置
    double insgps_x_;
    double insgps_y_;

    // 地图原点
    double latitude_;
    double longitude_;
    double height_;

    // 地图数据
    vector<positionConf> route_data_;
    // 等间距采点后的索引
    vector<int> map_index_;
    positionConf p_;

    // 采点距离
    double equal_length_;

    // flags
    int save_flag_;
    int start_record_flag_;

    // Subscriber and Publisher
    rclcpp::Subscription<location_msgs::msg::RTK>::SharedPtr rtk_sub_;
};


#endif