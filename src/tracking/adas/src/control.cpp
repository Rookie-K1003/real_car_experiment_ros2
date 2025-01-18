#include "control.h"

adas_control::adas_control(const rclcpp::NodeOptions & node_options) : Node("adas_control_node", node_options)
{
    RCLCPP_INFO(get_logger(), "adas_control initialization...");

    this->declare_parameter<double>("/calibration/insgps_x", 0.0);
    insgps_x_ = this->get_parameter("/calibration/insgps_x").get_parameter_value().get<double>();

    this->declare_parameter<double>("/calibration/insgps_y", 0.0);
    insgps_y_ = this->get_parameter("/calibration/insgps_y").get_parameter_value().get<double>();

    this->declare_parameter<double>("/calibration/center2frontaxis", 0.0);
    center2frontaxis_ = this->get_parameter("/calibration/center2frontaxis").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/latitude", 0.0);
    latitude_ = this->get_parameter("/map/latitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/longitude", 0.0);
    longitude_ = this->get_parameter("/map/longitude").get_parameter_value().get<double>();

    this->declare_parameter<double>("/map/height", 0.0);
    height_ = this->get_parameter("/map/height").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/preview_time", 0.0);
    preview_time_ = this->get_parameter("/control/preview_time").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/basic_prelength_min", 0.0);
    basic_prelength_min_ = this->get_parameter("/control/basic_prelength_min").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/basic_prelength_max", 0.0);
    basic_prelength_max_ = this->get_parameter("/control/basic_prelength_max").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/Rmax", 0.0);
    Rmax_ = this->get_parameter("/control/Rmax").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/lamda_preview", 0.0);
    lamda_preview_ = this->get_parameter("/control/lamda_preview").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/lamda_stanley", 0.0);
    lamda_stanley_ = this->get_parameter("/control/lamda_stanley").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/k_stanley", 0.0);
    k_stanley_ = this->get_parameter("/control/k_stanley").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/k_soft", 0.0);
    k_soft_ = this->get_parameter("/control/k_soft").get_parameter_value().get<double>();

    this->declare_parameter<int>("/control/turnleft_ispositive", 0);
    turnleft_ispositive_ = this->get_parameter("/control/turnleft_ispositive").get_parameter_value().get<int>();

    this->declare_parameter<double>("/control/turn_ratio", 0.0);
    turn_ratio_ = this->get_parameter("/control/turn_ratio").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/max_turn_angle", 0.0);
    max_turn_angle_ = this->get_parameter("/control/max_turn_angle").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/max_speed", 0.0);
    max_speed_ = this->get_parameter("/control/max_speed").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/sys_delay", 0.0);
    sys_delay_ = this->get_parameter("/control/sys_delay").get_parameter_value().get<double>();


    pid_control_angle_ = std::make_shared<PIDControl>(rclcpp::NodeOptions{});
    pid_control_speed_ = std::make_shared<PIDControl>(rclcpp::NodeOptions{});

    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("ogm_map", 1, std::bind(&adas_control::callbackOccupancyGrid, this, std::placeholders::_1));

    // 发送
    control_pub_ = create_publisher<control_msgs::msg::ControlReq>("/tracking_control_req", 10);
    Pose_car_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rtk_car", 1);
    Pose_close_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rtk_close_point", 1);
    Pose_pre_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rtk_pre_point", 1);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1);

    close_index_ = -1;
    finish_index_ = 0;

    // 设置PID参数
    // 横向控制
    this->declare_parameter<double>("/control/lateral/kp", 0.0);
    double lateral_kp = this->get_parameter("/control/lateral/kp").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/lateral/ki", 0.0);
    double lateral_ki = this->get_parameter("/control/lateral/ki").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/lateral/kd", 0.0);
    double lateral_kd = this->get_parameter("/control/lateral/kd").get_parameter_value().get<double>();

    pid_conf_angle_.kp = lateral_kp;
    pid_conf_angle_.ki = lateral_ki;
    pid_conf_angle_.kd = lateral_kd;
    pid_conf_angle_.kaw = 0;
    pid_control_angle_->init(pid_conf_angle_);

    // 速度控制
    this->declare_parameter<double>("/control/velocity/kp", 0.0);
    double velocity_kp = this->get_parameter("/control/velocity/kp").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/velocity/ki", 0.0);
    double velocity_ki = this->get_parameter("/control/velocity/ki").get_parameter_value().get<double>();

    this->declare_parameter<double>("/control/velocity/kd", 0.0);
    double velocity_kd = this->get_parameter("/control/velocity/kd").get_parameter_value().get<double>();

    pid_conf_speed_.kp = velocity_kp;
    pid_conf_speed_.ki = velocity_ki;
    pid_conf_speed_.kd = velocity_kd;
    pid_conf_speed_.kaw = 0;
    pid_control_speed_->init(pid_conf_speed_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "adas_control initialization finished.");
}

void adas_control::callbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    GridMapData.clear();
    half_width = -msg->info.origin.position.y;
    half_length = -msg->info.origin.position.x;
    resolution = msg->info.resolution;
    for (unsigned int i = 0; i < msg->data.size(); i++)
    {
        if (abs(msg->data.at(i) - 0.0) < 1e-5)
            GridMapData.push_back(0);
        else
        {
            GridMapData.push_back(1);
        }
        // std::cout << "index and i and j in car: " << i <<"  " << half_width - floor(i / (2*half_length/resolution))*resolution << " "  << -half_length + (i % int(ceil(2*half_length/resolution)))*resolution << "  " << GridMapData.at(i) << "\n"; 
    }
    // double left = -1.0, right = 1.0, back = 2.0, front = 10.0;
    // for(int i = floor((half_width - right)/resolution); i <= floor((half_width - left)/resolution); i++)
    //     for(int j = floor((back + half_length)/resolution); j <= floor((front + half_length)/resolution); j++)
    //         {
    //             int index = i * int(ceil(2*half_length / resolution)) + j;
    //             // std::cout << "check index and i and j in car: " << index <<"  " << half_width - floor(index / (2*half_length/resolution))*resolution << " "  << -half_length + (index % int(ceil(2*half_length/resolution)))*resolution << "\n"; 
    //             // std::cout << "i and j and index and grid : " << i << "  " << j << "  " <<  <<"   " << GridMapData[i * floor(2*half_length / resolution) + j] << "\n";
    //             if(GridMapData[index])
    //             {
    //                 std::cout << "check index and i and j in car: " << index <<"  " << half_width - floor(index / (2*half_length/resolution))*resolution << " "  << -half_length + (index % int(ceil(2*half_length/resolution)))*resolution << "\n"; 
    //             }
    //         }
    // std::cout <<"end!!" <<"\n";
}

bool adas_control::checkObstacle(const positionConf &car_, const vector<positionConf> &route_data_)
{
    // int current_index_ = close_index_;
    // positionConf route_new;
    // while(sqrt(pointDistanceSquare(car_, route_data_[current_index_]) < 15))
    // {
    //     //将轨迹点转到车辆坐标系
    //     if (sqrt(pointDistanceSquare(car_, route_data_[current_index_]) < 2))
    //     {
    //         current_index_ ++;
    //         continue;
    //     }
    //     route_new.x = cos(car_.heading)*route_data_[current_index_].x + 
    //         sin(car_.heading)*route_data_[current_index_].y + car_.x;
    //     route_new.y = -sin(car_.heading)*route_data_[current_index_].x + 
    //         cos(car_.heading)*route_data_[current_index_].y + car_.y;
    //     //再平移到栅格坐标系
    //     route_new.x += half_width;
    //     route_new.y += half_length;
    //     //计算对应的栅格位置
    //     int col = floor(route_new.x / resolution);
    //     int row = floor(route_new.y / resolution);
    //     int index = row * 2 * floor(half_length / resolution) + col;
    //     //判断该栅格和边上栅格的占据状态;/
    //     int n = 2;//可调参数，n*栅格分辨率=横向避障距离
    //     for(int i=0;i<=n;i++)
    //     {
    //         if(GridMapData[index+i] || GridMapData[index-i])
    //             return true;
    //         else
    //             continue;
    //     }
    //     current_index_ ++;
    // }
    // return false;
    double left = -1.0, right = 1.0, back = 2.0, front = 10.0;
    for(int i = floor((half_width - right)/resolution); i <= floor((half_width - left)/resolution); i++)
        for(int j = floor((back + half_length)/resolution); j <= floor((front + half_length)/resolution); j++)
            {
                if(i * int(ceil(2*half_length / resolution)) + j >= GridMapData.size()) return false;
                if(GridMapData.at(i * int(ceil(2*half_length / resolution)) + j))
                {
                    std::cout << "i and j and index and grid : " << i << "  " << j << "  " << i + j*2 * floor(half_width / resolution) <<"   " << GridMapData[i * floor(2*half_length / resolution) + j] <<                                "\n";
                    return true;
                }
            }
    return false;
}

void adas_control::cal_control(const positionConf &car_, const vector<positionConf> &route_data_)
{
    if (car_.longitude == 0)
        return;
    // todo
    // rclcpp::spin_some(node);

    finish_index_ = route_data_.size();
    //临时变量
    positionConf car_lla = {0};      //组合导航传来的组合导航经纬度位姿
    positionConf pre_pos = {0};      //预瞄点的XYZ位姿
    positionConf diag_pre_pos = {0}; //判断斜行点位姿
    positionConf xy_pos = {0};       //车辆中心全局XYZ位姿
    positionConf xy_pos_fc = {0};    //车辆前轴中心全局XYZ位姿

    // v-车辆坐标系...
    // n-最近点...
    // p-预瞄点...
    positionConf vnxy_pos = {0};    //距车辆中心最近点的xyz车辆坐标系位姿
    positionConf vnxy_pos_fc = {0}; //距车辆前轴中心最近点的xyz车辆坐标系位姿
    positionConf vpxy_pos = {0};    //预瞄点的xyz车辆坐标系位姿

    //car_lla = car_
    // printf("lla: %lf,%lf,%lf \n", car_.longitude, car_.latitude, car_.height);
    positionConf_copy(car_, xy_pos);

    // 坐标变换 LLA坐标->车辆中心全局XYZ位姿
    gps2xy(xy_pos, longitude_, latitude_, height_);
    // insgps2center(xy_pos, insgps_x_, insgps_y_);

    // printf("x : %lf, y : %lf, heading : %lf \n", xy_pos.x, xy_pos.y, xy_pos.heading);

    // 找最近点
    // close_index_ 理论上的车辆中心最近点,作为成员变量存储做下一帧的先验
    findClosestPoint(route_data_, close_index_, xy_pos);
    center2front(xy_pos, xy_pos_fc);

    int close_index_theory_fc = close_index_;
    findClosestPoint(route_data_, close_index_theory_fc, xy_pos_fc);

    // 考虑系统延迟推算真正的最近点以及计算曲率半径的点
    // 实际上的车辆中心和前轴中心最近点
    int close_index;
    int close_index_fc;
    close_index = findRealCloseIndex(close_index_, xy_pos, route_data_);
    close_index_fc = findRealCloseIndex(close_index_theory_fc, xy_pos_fc, route_data_);

    int index_r1, index_f1, index_f2, index_f3;
    findPoints2ComputeRadius(close_index, index_r1, index_f1, index_f2, index_f3, route_data_);

    // 计算曲率半径
    double R1 = getR(route_data_[close_index], route_data_[index_r1], route_data_[index_f1]);
    double R4 = getR(route_data_[close_index], route_data_[index_f1], route_data_[index_f2]);
    double R5 = getR(route_data_[index_f1], route_data_[index_f2], route_data_[index_f3]);
    // R = min(R1, R4, R5)
    double R = R1;
    R = R < R4 ? R : R4;
    R = R < R5 ? R : R5;

    // 计算预瞄点
    // diag_pre_pos 没用上的
    double pre_length = 5;
    findPrePoint(diag_pre_pos, close_index, pre_length, route_data_);

    pre_length = (2 * (basic_prelength_max_ - basic_prelength_min_)) / (1 + exp(-(R - Rmax_))) + basic_prelength_min_ + xy_pos.velocity * preview_time_;
    findPrePoint(pre_pos, close_index, pre_length, route_data_);

    //将两个最近点和预瞄点的坐标转换到车辆坐标系下
    enu2veh(vnxy_pos, route_data_[close_index], xy_pos);
    enu2veh(vnxy_pos_fc, route_data_[close_index_fc], xy_pos);
    enu2veh(vpxy_pos, pre_pos, xy_pos);

    //横向控制模型计算横向误差
    double latteral_error = StanleyPreviewLatteralModel(xy_pos, vnxy_pos, vnxy_pos_fc, vpxy_pos);

    // 角度输出
    control_angle_ = pid_control_angle_->control(latteral_error, 0.01);
    previous_control_angle_ = control_angle_;
    control_angle_ = control_angle_ > max_turn_angle_ ? max_turn_angle_ : control_angle_;
    control_angle_ = control_angle_ < (-max_turn_angle_) ? (-max_turn_angle_) : control_angle_;

    // 速度输出
    std::cout << "obstacle: " << checkObstacle(car_,route_data_) << "\n";
    control_speed_ = checkObstacle(car_,route_data_) ? 0 : route_data_[close_index].velocity;
    // control_speed_ = route_data_[close_index].velocity;

    // visulization
    visulization(xy_pos, route_data_[close_index], pre_pos);

    // pub dwa goal

    Eigen::Quaternion<double> q(1, 0, 0, 0);
    goal_pose_.header.frame_id = "/map";

    // close to the goal, do not need avoidance
    if (close_index + 100 >= route_data_.size())
        return;
    q = Eigen::AngleAxisd(route_data_[close_index + 100].heading * M_PI / 180, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(route_data_[close_index + 100].roll * M_PI / 180, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(route_data_[close_index + 100].pitch * M_PI / 180, Eigen::Vector3d::UnitX());
    goal_pose_.pose.orientation.x = q.x();
    goal_pose_.pose.orientation.y = q.y();
    goal_pose_.pose.orientation.z = q.z();
    goal_pose_.pose.orientation.w = q.w();
    goal_pose_.pose.position.x = route_data_[close_index + 100].x;
    goal_pose_.pose.position.y = route_data_[close_index + 100].y;
    goal_pose_.pose.position.z = route_data_[close_index + 100].z;
    goal_pub_->publish(goal_pose_);
}

void adas_control::send_control()
{
    if (finish_index_ == 0)
        return;
    control_speed_ = control_speed_ > max_speed_ ? max_speed_ : control_speed_;
    control_speed_ = control_speed_ < 0 ? 0 : control_speed_;

    control_msgs::msg::ControlReq control_cmd;
    control_cmd.vel_req = control_speed_;
    control_cmd.angle_req = control_angle_;
    if (std::abs(control_speed_ - 0) <= 10e-5)
        control_cmd.gear_req = 2;
    else if (control_speed_ < 0)
        control_cmd.gear_req = 1;
    else
        control_cmd.gear_req = 3;
    cout<<control_cmd.gear_req<<endl;
    // 到达终点
    if (finish_index_ - close_index_ <= 3)
    {
        control_cmd.vel_req = 0;
        control_cmd.angle_req = 0;
        control_cmd.gear_req = 2;
        RCLCPP_INFO(get_logger(), "Arrived the ending point !!! ...");
    }

    RCLCPP_INFO(get_logger(), "control angle: %.4lf, control_speed :%.4lf,control_gear :%d", control_cmd.angle_req, control_cmd.vel_req, control_cmd.gear_req);

    // publish
    control_pub_->publish(control_cmd);
}

void adas_control::findClosestPoint(const vector<positionConf> &data, int &prior_index, const positionConf &p)
{
    int search_index_up = data.size() - 1;
    int search_index_down = 0;
    if (prior_index != -1)
    {
        // 如果有先验
        // DEBUG search_range参数可能需要调整
        int search_range = 200;
        search_index_down = prior_index - search_range > 0 ? prior_index - search_range : 0;
        search_index_up = prior_index + search_range < data.size() - 1 ? prior_index + search_range : data.size() - 1;
    }

    double min_distance = 4000000; // 20m
    double distance = 0;
    for (int i = search_index_down; i <= search_index_up; i++)
    {
        distance = pointDistanceSquare(p, data[i]);
        prior_index = distance < min_distance ? i : prior_index;
        min_distance = distance < min_distance ? distance : min_distance;
    }

    if (min_distance == 4000000)
    {
        RCLCPP_WARN(get_logger(), "can't find nearest point in 20m !!!");
        exit(1);
    }
    else
    {
        // RCLCPP_INFO(get_logger(), "find nearest point and distance^2 : %lf", distance);
    }
}

double adas_control::pointDistanceSquare(const positionConf &p1, const positionConf &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

void adas_control::center2front(const positionConf &center, positionConf &front)
{
    front.x = center.x + center2frontaxis_ * sin(center.heading * M_PI / 180);
    front.y = center.y + center2frontaxis_ * cos(center.heading * M_PI / 180);
}

int adas_control::findRealCloseIndex(const int &close_index_theory, const positionConf &p, const vector<positionConf> &route_data_)
{
    double delay_length = p.velocity * sys_delay_;
    double l = 0;

    int down = close_index_theory;
    int up = close_index_theory;
    while (l < delay_length)
    {
        if (down == route_data_.size() - 1)
            break;
        up = down + 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        down++;
    }
    return up;
}

// 找计算曲率半径的点
// 在路径中，在当前点左边找一个间距点，右边找三个间距点，间距范围为dl
void adas_control::findPoints2ComputeRadius(const int &close_index, int &index_r1,
                                           int &index_f1, int &index_f2, int &index_f3,
                                           const vector<positionConf> &route_data_)
{
    // DEBUG
    // double equal_l = (center2frontaxis + center2rearaxis)/2;
    // 按照上面公式才0.8..
    double equal_l = 2;
    double l = 0;
    int down, up;

    // find rear point
    l = 0;
    down = close_index;
    up = close_index;
    while (l < equal_l)
    {
        if (up == 0)
            break;
        down = up - 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        up--;
    }
    index_r1 = down;

    // find front point 1;
    l = 0;
    down = close_index;
    up = close_index;
    while (l < equal_l)
    {
        if (down == route_data_.size() - 1)
            break;
        up = down + 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        down++;
    }
    index_f1 = up;

    // find front point 2;
    l = 0;
    down = close_index;
    up = close_index;
    while (l < equal_l * 2)
    {
        if (down == route_data_.size() - 1)
            break;
        up = down + 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        down++;
    }
    index_f2 = up;

    // find front point 3;
    l = 0;
    down = close_index;
    up = close_index;
    while (l < equal_l * 3)
    {
        if (down == route_data_.size() - 1)
            break;
        up = down + 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        down++;
    }
    index_f3 = up;
}

double adas_control::getR(const positionConf &pt1, const positionConf &pt2, const positionConf &pt3)
{
    double a = 2 * (pt2.x - pt1.x);
    double b = 2 * (pt2.y - pt1.y);
    double c = pt2.x * pt2.x + pt2.y * pt2.y - pt1.x * pt1.x - pt1.y * pt1.y;
    double d = 2 * (pt3.x - pt2.x);
    double ee = 2 * (pt3.y - pt2.y);
    double f = pt3.x * pt3.x + pt3.y * pt3.y - pt2.x * pt2.x - pt2.y * pt2.y;
    double r;
    if ((b * d - ee * a) == 0)
    {
        r = Rmax_;
    }
    else
    {
        double x = (b * f - ee * c) / (b * d - ee * a);
        double y = (d * c - a * f) / (b * d - ee * a);
        r = sqrt((x - pt1.x) * (x - pt1.x) + (y - pt1.y) * (y - pt1.y));
        if (r >= Rmax_)
        {
            r = Rmax_;
        }
    }

    return r;
}

void adas_control::findPrePoint(positionConf &pre_p, const int &index, const double &pre_length, const vector<positionConf> &route_data_)
{
    double l = 0;
    int down = index;
    int up = index;
    while (l < pre_length)
    {
        if (down == route_data_.size() - 1)
            break;
        up = down + 1;
        l += sqrt(pointDistanceSquare(route_data_[down], route_data_[up]));
        down++;
    }

    positionConf_copy(route_data_[up], pre_p);
}

void adas_control::enu2veh(positionConf &p_veh, const positionConf &p, const positionConf &xy_p)
{
    positionConf_copy(p, p_veh);
    p_veh.x = (p.x - xy_p.x) * cos(xy_p.heading * M_PI / 180) -
              (p.y - xy_p.y) * sin(xy_p.heading * M_PI / 180);
    p_veh.y = (p.x - xy_p.x) * sin(xy_p.heading * M_PI / 180) +
              (p.y - xy_p.y) * cos(xy_p.heading * M_PI / 180);
    p_veh.z = p.z;
}

double adas_control::StanleyPreviewLatteralModel(const positionConf &xy_p, const positionConf &vnxy_p,
                                                 const positionConf &vnxy_p_fc, const positionConf &vpxy_p)
{
    double degree_error = 0;

    // yaw为正, 最近点vnxy_p.heading顺时针旋转yaw角到当前点xy_p.heading,方向盘左转
    double yaw = computedheading(xy_p.heading, vnxy_p.heading);

    // 预瞄点vpxy_p在车辆坐标系左前方,preyaw为正,方向盘左转
    double preyaw = -atan2(vpxy_p.x, vpxy_p.y) * 180 / M_PI;

    degree_error = yaw + lamda_preview_ * preyaw + lamda_stanley_ * atan2(-k_stanley_ * vnxy_p_fc.x, (k_soft_ + xy_p.velocity)) * 180 / M_PI;
    degree_error *= turnleft_ispositive_;
    return degree_error;
}

double adas_control::computedheading(const double &heading1, const double &heading2)
{
    double dheading = 0;
    if (heading1 > (heading2 + 180))
    {
        dheading = heading1 - heading2 - 360;
    }
    else if (heading1 < (heading2 - 180))
    {
        dheading = 360 - (heading2 - heading1);
    }
    else
    {
        dheading = heading1 - heading2;
    }

    return dheading;
}

// 可视化 地图-车辆点-最近点-预瞄点
void adas_control::visulization(const positionConf &xy_pos, const positionConf &close_pos, const positionConf &pre_pos)
{
    Eigen::Quaternion<double> q(1, 0, 0, 0);
    nav_msgs::msg::Odometry map_pose;
    map_pose.header.frame_id = "/map";
    map_pose.header.stamp = this->now();
    q = Eigen::AngleAxisd(close_pos.heading * M_PI / 180, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(close_pos.roll * M_PI / 180, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(close_pos.pitch * M_PI / 180, Eigen::Vector3d::UnitX());
    map_pose.pose.pose.orientation.x = q.x();
    map_pose.pose.pose.orientation.y = q.y();
    map_pose.pose.pose.orientation.z = q.z();
    map_pose.pose.pose.orientation.w = q.w();
    map_pose.pose.pose.position.x = close_pos.x;
    map_pose.pose.pose.position.y = close_pos.y;
    map_pose.pose.pose.position.z = close_pos.z;
    Pose_close_pub_->publish(map_pose);

    q = Eigen::AngleAxisd(pre_pos.heading * M_PI / 180, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pre_pos.roll * M_PI / 180, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pre_pos.pitch * M_PI / 180, Eigen::Vector3d::UnitX());
    map_pose.pose.pose.orientation.x = q.x();
    map_pose.pose.pose.orientation.y = q.y();
    map_pose.pose.pose.orientation.z = q.z();
    map_pose.pose.pose.orientation.w = q.w();
    map_pose.pose.pose.position.x = pre_pos.x;
    map_pose.pose.pose.position.y = pre_pos.y;
    map_pose.pose.pose.position.z = pre_pos.z;
    Pose_pre_pub_->publish(map_pose);

    // 东北天的heading到xyz heading的转化
    //! 注意 这里发布出去的车辆坐标系tf，前x左y
    double transform_heading = (-xy_pos.heading + 90.0) > 0 ? (-xy_pos.heading + 90.0) : (-xy_pos.heading + 90.0 + 360.0);
    transform_heading = transform_heading / 180.0 * M_PI;

    //todo shang mian de dou yao gai
    geometry_msgs::msg::Quaternion quaternion_msg;
    tf2::Quaternion q_temp;
    q_temp.setRPY(0, 0, transform_heading);
    quaternion_msg = tf2::toMsg(q_temp);

    map_pose.pose.pose.orientation = quaternion_msg;
    map_pose.pose.pose.position.x = xy_pos.x;
    map_pose.pose.pose.position.y = xy_pos.y;
    map_pose.pose.pose.position.z = xy_pos.z;
    Pose_car_pub_->publish(map_pose);

    // pub tf

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp =  rclcpp::Clock ().now ();
    transform_stamped.header.frame_id = "/map";
    transform_stamped.child_frame_id = "/rslidar";
    transform_stamped.transform.translation.x = xy_pos.x;
    transform_stamped.transform.translation.y = xy_pos.y;
    transform_stamped.transform.translation.z = xy_pos.z;
    transform_stamped.transform.rotation.x = q_temp.x();
    transform_stamped.transform.rotation.y = q_temp.y();
    transform_stamped.transform.rotation.z = q_temp.z();
    transform_stamped.transform.rotation.w = q_temp.w();

    tf_broadcaster_->sendTransform(transform_stamped);
}