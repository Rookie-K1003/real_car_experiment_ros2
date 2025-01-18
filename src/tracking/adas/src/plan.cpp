#include "plan.h"

adas_plan::adas_plan(const rclcpp::NodeOptions & node_options) : Node("adas_plan_node", node_options)
{
    RCLCPP_INFO(get_logger(), "adas_plan initialization...");

    this->declare_parameter<double>("/map/latitude", 0.0);
    latitude_ = this->get_parameter("/map/latitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/longitude", 0.0);
    longitude_ = this->get_parameter("/map/longitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/height", 0.0);
    height_ = this->get_parameter("/map/height").get_parameter_value().get<double>();

    // 读取地图
    getRouteFromFile();

    // 接收 & 发送
    rtk_sub_ = this->create_subscription<location_msgs::msg::RTK>("rtk_data", 10, std::bind(&adas_plan::recvHuacePosCallback, this, std::placeholders::_1));
    map_pub_ = create_publisher<nav_msgs::msg::Path>("/rtk_map", 1);


    // init
    car_.velocity = 0;
    car_.heading = 0;
    car_.pitch = 0;
    car_.roll = 0;
    car_.longitude = 0;
    car_.latitude = 0;
    car_.height = 0;
    car_.x = 0;
    car_.y = 0;
    car_.z = 0;
    car_.gpstime = 0;
    car_.dist = 0;

    RCLCPP_INFO(get_logger(), "adas_plan Initialization finished.");
}

void adas_plan::getRouteFromFile()
{
    string root_dir = ROOT_DIR;
    string file_name = DATA_NAME_REAL_ROUTE;
    string path = root_dir + "/../map/" + file_name;
    RCLCPP_INFO(get_logger(), "route date path name:%s", path);

    FILE *route_data_read_fp = NULL;
    route_data_read_fp = fopen(path.c_str(), "r");

    if (route_data_read_fp == NULL)
    {
        RCLCPP_WARN(get_logger(), "fail to open file !!!");
        exit(1);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Map file is open!");
        positionConf p;
        while (!feof(route_data_read_fp))
        {
            fscanf(route_data_read_fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &p.gpstime,
                   &p.x,
                   &p.y,
                   &p.z,
                   &p.longitude,
                   &p.latitude,
                   &p.height,
                   &p.heading,
                   &p.pitch,
                   &p.roll,
                   &p.velocity,
                   &p.dist);
            route_data_.push_back(p);
        }
        // 去除最后一个空行
        route_data_.pop_back();
        fclose(route_data_read_fp);
        // DEBUG
        printf("read %lu row, route_date is ok!\n", route_data_.size());
    }

    // for visulization
    geometry_msgs::msg::PoseStamped map_pose;
    Eigen::Quaternion<double> q(1, 0, 0, 0);
    for (int i = 0; i < route_data_.size(); i++)
    {
        map_pose.header.frame_id = "/map";
        //! 注意 这里发布出去的车辆坐标系tf，前x左y
        double transform_heading = (-route_data_[i].heading + 90.0) > 0 ? (-route_data_[i].heading + 90.0) : (-route_data_[i].heading + 90.0 + 360.0);
        transform_heading = transform_heading / 180.0 * M_PI;

        //todo shang mian de dou yao gai
        geometry_msgs::msg::Quaternion quaternion_msg;
        tf2::Quaternion q;
        q.setRPY(0, 0, transform_heading);
        quaternion_msg = tf2::toMsg(q);
        map_pose.pose.orientation = quaternion_msg;
        map_pose.pose.position.x = route_data_[i].x;
        map_pose.pose.position.y = route_data_[i].y;
        map_pose.pose.position.z = route_data_[i].z;
        visu_map_.poses.push_back(map_pose);
    }
    visu_map_.header.frame_id = "/map";
}

void adas_plan::recvHuacePosCallback(const location_msgs::msg::RTK::ConstSharedPtr msg)
{
    // rtk msg to struct p
    car_.velocity = msg->velocity;

    car_.heading = msg->heading;
    car_.pitch = msg->pitch;
    car_.roll = msg->roll;

    car_.longitude = msg->longitude;
    car_.latitude = msg->latitude;
    car_.height = msg->height;

    car_.gpstime = msg->gpstime;
    RCLCPP_INFO(get_logger(), "car_.gpstime:%f",car_.gpstime);
}

void adas_plan::visulization()
{
    visu_map_.header.stamp = this->now();
    map_pub_->publish(visu_map_);
}