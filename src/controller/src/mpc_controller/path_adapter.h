#ifndef PATH_ADAPTER_H__
#define PATH_ADAPTER_H__

#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <ecl/geometry.hpp>
#include <ecl/geometry/polynomial.hpp>
#include "geometry_msgs/msg/point.hpp"

/*
文件名: MPC控制器与参考轨迹的适配模块
作者: kq
完成时间: 2025.03.14

编译类型: 动态库

依赖: 
    ros2相关：
      rclcpp
      nav_msgs
      geometry_msgs
    外部库:
      math/quntic_solver
      ecl
*/


namespace Controller
{
    using ecl::CubicSpline;
    using geometry_msgs::msg::Point;
    using nav_msgs::msg::Path;

    // 自定义一个位姿结构体，便于配合使用相关函数
    struct PoseStamped {
        double x, y, yaw;
        double v;
        double t;
    };

    class PathAdapter
    {
    private:
        std::vector<Point> waypoints_;
        CubicSpline spline_x_;
        CubicSpline spline_y_;
        double spline_max_t_;

        void computeSplineWithPoints();
        double nearestPointOnSpline(double x, double y, double eps = 1e-8);
        double lengthOfSpline(double a, double b);
        double pointWithFixedDistance(double t, double d, double eps = 1e-8);
        double maxCurvature(double l, double r, double numerical_eps = 1e-8);

        double randomFloat();
    public:
        PathAdapter();

        bool loadPath(std::string filename, double scale = 1);
        void getPath(const PoseStamped &cur_pose, double dt,
                     double v_ref, double v_min, double k, double max_brake_accel, 
                     int n, std::vector<PoseStamped> *path);
        inline std::vector<Point> waypoints() const { return waypoints_; }
        void LoadReferenceTrajectory(Path ref_traj);

    };

} // namespace Controller
#endif