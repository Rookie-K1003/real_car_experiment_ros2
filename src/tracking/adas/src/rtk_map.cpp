#include "rtk_map.h"

rtk_map::rtk_map(const rclcpp::NodeOptions & node_options) : Node("rtk_map_node", node_options)
{
    // load paramaters from yaml
    RCLCPP_INFO(get_logger(), "Start to load paramaters from yaml...");
    // not support in ros2 foxy, but can be used in ros2 galactic
    // insgps_x_ = this->declare_parameter<double>("/calibration/insgps_x");

    this->declare_parameter<double>("/calibration/insgps_x", 0.0);
    insgps_x_ = this->get_parameter("/calibration/insgps_x").get_parameter_value().get<double>();

    this->declare_parameter<double>("/calibration/insgps_y", 0.0);
    insgps_y_ = this->get_parameter("/calibration/insgps_y").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/latitude", 31.029311);
    latitude_ = this->get_parameter("/map/latitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/longitude", 121.442124);
    longitude_ = this->get_parameter("/map/longitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/height", 15.67);
    height_ = this->get_parameter("/map/height").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/equal_length", 0.1);
    equal_length_ = this->get_parameter("/map/equal_length").get_parameter_value().get<double>();

    RCLCPP_INFO(get_logger(), "Paramaters loading finished.");

    // flags
    save_flag_ = 0;
    start_record_flag_ = 0;

    // init
    p_.velocity = 0;
    p_.heading = 0;
    p_.pitch = 0;
    p_.roll = 0;
    p_.longitude = 0;
    p_.latitude = 0;
    p_.height = 0;
    p_.x = 0;
    p_.y = 0;
    p_.z = 0;
    p_.gpstime = 0;
    p_.dist = 0;

    rtk_sub_ = this->create_subscription<location_msgs::msg::RTK>("/rtk_data", 30, std::bind(&rtk_map::recvHuacePosCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Initialization finished and enter loop.");

    msgs2points();
}

void rtk_map::msgs2points()
{
    rclcpp::WallRate loop_rate(FRE);
    rclcpp::Node::SharedPtr node_(this);

    while (rclcpp::ok())
    {
        // 获取键盘输入
        int ret = get_char();
        if (ret == 115)//s
        {
            RCLCPP_INFO(get_logger(), "Start get route point from GPS");
            start_record_flag_ = 1;
        }
        if (ret == 99)//c
        {
            RCLCPP_INFO(get_logger(), "Save route file");
            start_record_flag_ = 0;
            save_flag_ = 1;
        }

        // 完成采集输出地图文件
        if (save_flag_ == 1)
        {
            RCLCPP_INFO(get_logger(), "save route data!\n");
            rtk_map::save2File();
            break;
        }

        // todo
        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }

    RCLCPP_INFO(get_logger(), "map saved!\n");
    rclcpp::shutdown();
}

void rtk_map::save2File()
{
    FILE *route_data_save_fp = NULL;
    char end1 = 0x0d; // "/n"
    char end2 = 0x0a;

    // 存储路径
    string root_dir = ROOT_DIR;
    string file_name = DATA_NAME_REAL_ROUTE;
    string path = root_dir + "/../map/" + file_name;
    RCLCPP_INFO(get_logger(), "route date path name:%s", path);

    route_data_save_fp = fopen(path.c_str(), "w+");
    if (route_data_save_fp == NULL)
    {
        RCLCPP_WARN(get_logger(), "fail to open file !!!");
        exit(1);
    }

    if (route_data_.size() == 0)
    {
        RCLCPP_WARN(get_logger(), "no data recorded!!!");
        exit(1);
    }

    // 计算里程
    route_data_[0].dist = 0;
    for (int i = 1; i < route_data_.size(); i++)
    {
        double dx = route_data_[i].x - route_data_[i - 1].x;
        double dy = route_data_[i].y - route_data_[i - 1].y;
        route_data_[i].dist = route_data_[i - 1].dist + sqrt(dx * dx + dy * dy);
    }

    // 等间距采点的索引
    map_index_.push_back(0);

    double dist = 0;
    int index = 1;
    while (index < route_data_.size())
    {
        dist = route_data_[index].dist - route_data_[map_index_.back()].dist;
        if (dist > equal_length_)
            map_index_.push_back(index);
        index++;
    }

    //写入文件
    printf("the num of points in map : %d \n", map_index_.size());
    for (int i = 0; i < map_index_.size(); i++)
    {
        int j = map_index_[i];
        fprintf(route_data_save_fp, "%.3lf %.2lf %.2lf %.2lf %.8lf %.8lf %.3lf %.2lf %.2lf %.2lf %.2lf %.2lf%c",
                route_data_[j].gpstime,
                route_data_[j].x,
                route_data_[j].y,
                route_data_[j].z,
                route_data_[j].longitude,
                route_data_[j].latitude,
                route_data_[j].height,
                route_data_[j].heading,
                route_data_[j].pitch,
                route_data_[j].roll,
                route_data_[j].velocity,
                route_data_[j].dist,
                end2);
    }

    fclose(route_data_save_fp);
}

int rtk_map::get_char()
{
    // TODO
    // 头文件？？
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar();
    }

    return ch;
}

void rtk_map::recvHuacePosCallback(const location_msgs::msg::RTK::ConstSharedPtr msg)
{
    // rtk msg to struct p
    //p_.velocity = msg->velocity;
    p_.velocity = 1;
    //RCLCPP_INFO(get_logger(), "vel : %lf", p_.velocity);
    p_.heading = msg->heading;
    p_.pitch = msg->pitch;
    p_.roll = msg->roll;

    p_.longitude = msg->longitude;
    p_.latitude = msg->latitude;
    p_.height = msg->height;

    p_.gpstime = msg->gpstime;

    // 采集地图点
    if (start_record_flag_ == 1)
    {
        gps2xy(p_, longitude_, latitude_, height_);
        insgps2center(p_, insgps_x_, insgps_y_);
        if (route_data_.size() == 0)
            route_data_.push_back(p_);
        else
        {
            if (p_.longitude != route_data_.back().longitude || p_.latitude != route_data_.back().latitude)
                route_data_.push_back(p_);
        }
        
    }
}
