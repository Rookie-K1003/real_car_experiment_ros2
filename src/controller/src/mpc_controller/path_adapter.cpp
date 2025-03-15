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

#include "path_adapter.h"
#include "quntic_solver.h"

namespace Controller
{
  PathAdapter::PathAdapter() {

  }

  bool PathAdapter::loadPath(std::string filename, double scale) 
  {
    waypoints_.clear();

    FILE *file = fopen(filename.c_str(), "rt");
    if(file != nullptr) {
        double x, y;
        while(fscanf(file, "%lf,%lf", &x, &y) != EOF) {
            Point point;

            point.x = scale * x;
            point.y = scale * y;
            point.z = 0;

            waypoints_.push_back(point);
        }
        fclose(file);

        computeSplineWithPoints();

        return true;
    }

    return false;
  }
  
  void PathAdapter::computeSplineWithPoints()
  {
    int num_waypoints = int(waypoints_.size());
      if(num_waypoints <= 0) {
          return;
      }

      //create x and y line array
      static ecl::Array<double> x_list, y_list, t_list;
      x_list.resize(num_waypoints);
      y_list.resize(num_waypoints);
      t_list.resize(num_waypoints);

      double t = 0;
      for(int i = 0; i < num_waypoints; i++) {
          x_list[i] = waypoints_[i].x;
          y_list[i] = waypoints_[i].y;
          t_list[i] = t;

          t += 1;
      }

      spline_max_t_ = t_list[num_waypoints - 1];

      //interpolate parametric cubic spline
      spline_x_ = CubicSpline::Natural(t_list, x_list);
      spline_y_ = CubicSpline::Natural(t_list, y_list);

  }

  /*
    Assume that a, b >= 0
  */
  /*
    用于计算给定参数a和b之间某条样条曲线（spline）的长度。
    这里假设样条曲线由多个三次多项式（cubic polynomials）组成，每个多项式代表曲线的一个片段（segment）。
    函数主要使用了高斯求积（Gaussian quadrature）的方法来近似计算曲线长度。
  */
  double PathAdapter::lengthOfSpline(double a, double b) 
  {
      const double gauss_quadrature_coeff[5][2] = {
          {  0.0,                0.5688888888888889 },
          { -0.5384693101056831, 0.47862867049936647 },
          {  0.5384693101056831, 0.47862867049936647 },
          { -0.906179845938664,  0.23692688505618908 },
          {  0.906179845938664,  0.23692688505618908 }
      };

      double length = 0;
      int max_segments = int(spline_x_.polynomials().size()) - 1;
      for(int segment = int(a); segment <= std::min(int(b), max_segments); segment++) {
          double start = std::max(a, double(segment));
          double end   = std::min(b, double(segment + 1));

          //get coefficients
          const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
          const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

          //integrate this segment
          double integral = 0;
          for(int k = 0; k < 5; k++) {
              double x = ((end - start) / 2) * gauss_quadrature_coeff[k][0] + ((start + end) / 2);

              double A_x = coeff_x[1] + 2 * coeff_x[2] * x + 3 * coeff_x[3] * x * x;
              double B_x = coeff_y[1] + 2 * coeff_y[2] * x + 3 * coeff_y[3] * x * x;
              double F_x = std::sqrt( A_x * A_x + B_x * B_x );

              integral += gauss_quadrature_coeff[k][1] * F_x;
          }
          integral *= (end - start) / 2;

          length += integral;
      }

      return length;
  }

  /*
    在给定的二维平面点(x, y)上找到一条三次样条曲线（spline）上距离该点最近的点，
    并返回该点在样条曲线上的参数t。
  */
  double PathAdapter::nearestPointOnSpline(double x, double y, double eps) 
  {
      double min_dist_2 = 1.0 / 0.0;
      double min_t = 0;

      for(int segment = 0; segment < spline_max_t_; segment++) {
          //get 5th polynomial
          /*
              evaluated using sagemath

              a, b, c, d = var("a, b, c, d")
              e, f, g, h = var("e, f, g, h")
              x, y = var("x, y")

              f_x(t) = a + b * t + c * t^2 + d * t^3
              f_y(t) = e + f * t + g * t^2 + h * t^3

              df_x(t) = b + 2*c*t + 3*d*t^2
              df_y(t) = f + 2*g*t + 3*h*t^2

              -(
                  (x - f_x(t)) * df_x(t) + (y - f_y(t)) * df_y(t)
              ).expand().collect(t)
          */

          const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
          const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

          double a = coeff_x[0];
          double b = coeff_x[1];
          double c = coeff_x[2];
          double d = coeff_x[3];

          double e = coeff_y[0];
          double f = coeff_y[1];
          double g = coeff_y[2];
          double h = coeff_y[3];

          QuinticSolver::QuinticPolynomial poly = {
              a*b + e*f - b*x - f*y, //0
              (b*b + 2*a*c + f*f + 2*e*g - 2*c*x - 2*g*y), //1
              3*(b*c + a*d + f*g + e*h - d*x - h*y), //2
              2*(c*c + 2*b*d + g*g + 2*f*h), //3
              5*(c*d + g*h), //4
              3*(d*d + h*h) //5
          };

          //solve 5th polynomial
          std::vector<double> sol_t;
          QuinticSolver::solveRealRootsSpecialForCubicSplineDerivative(poly, double(segment), double(segment + 1), &sol_t, eps);

          //get nearest
          for(auto it = sol_t.begin(); it != sol_t.end(); it++) {
              double t  = *it;
              double dx = x - spline_x_(t);
              double dy = y - spline_y_(t);
              double dist_2 = dx * dx + dy * dy;

              if(dist_2 < min_dist_2) {
                  min_dist_2 = dist_2;
                  min_t = t;
              }
          }
      }

      return min_t;
  }

  /*
      旨在找到一条样条曲线（spline）上距离给定起始点t固定距离d的点，并返回该点在样条曲线上的参数值（通常是一个时间参数t）。
      这个函数使用了二分查找（bi-section）算法来实现这一目的，并且考虑了精度eps来避免无限循环。
  */
  double PathAdapter::pointWithFixedDistance(double t, double d, double eps) {
      if(d < eps) return t;

      //bi-section
      double l = t;
      double r = spline_max_t_;

      while((r - l) > eps) {
          double m = (l + r) / 2;

          if(lengthOfSpline(t, m) <= d) l = m;
          else r = m;
      }

      return (l + r) / 2;
  }

  double PathAdapter::maxCurvature(double l, double r, double numerical_eps) {
      double max_curvature = 0;

      int max_segments = int(spline_x_.polynomials().size()) - 1;
      for(int segment = int(l); segment <= std::min(int(r), max_segments); segment++) {
          double start = std::max(l, double(segment));
          double end   = std::min(r, double(segment + 1));

          const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
          const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

          //quadratic equation of curvature = a * x^2 + b * x + c
          double a = 36 * (coeff_x[3] * coeff_x[3] + coeff_y[3] * coeff_y[3]);
          double b = 24 * (coeff_x[2] * coeff_x[3] + coeff_y[2] * coeff_y[3]);
          double c = 4  * (coeff_x[2] * coeff_x[2] + coeff_y[2] * coeff_y[2]);

          if(std::fabs(b) > numerical_eps) {
              double x = -a / b;
              if((x >= start) && (x <= end)) max_curvature = std::max(max_curvature, c + x * (b + x * a));
          }

          max_curvature = std::max(max_curvature, c + start * (b + start * a));
          max_curvature = std::max(max_curvature, c +   end * (b + end   * a));
      }

      return max_curvature;
  }

  /**
   * @brief 根据当前位置、速度和参数计算路径
   *
   * 根据当前位置、速度、参考速度、最小速度、曲率系数、最大刹车加速度、步数和路径向量，计算并填充路径向量。
   *
   * @param cur_pose 当前位置信息
   * @param dt 时间步长
   * @param v_ref 参考速度
   * @param v_min 最小速度
   * @param k 曲率系数
   * @param max_brake_accel 最大刹车加速度,传入绝对值
   * @param n 步数
   * @param path 路径向量，用于存储计算得到的路径
   */
  void PathAdapter::getPath(const PoseStamped &cur_pose, double dt, double v_ref, double v_min, double k, double max_brake_accel, int n, std::vector<PoseStamped> *path) {
      //initial pose
      path->clear();

      //find nearest point on trajectory
      double t_start = nearestPointOnSpline(cur_pose.x, cur_pose.y);

      //calculate max curvature radius
      /*double brake_time = std::max(v_min, cur_pose.v) / max_brake_accel;
      double horizon_length = cur_pose.v * brake_time - 0.5 * max_brake_accel * brake_time * brake_time;
      double max_curvature = maxCurvature(t_start, pointWithFixedDistance(t_start, horizon_length));

      //calculate speed
      double eps = 1e-8;
      double v   = v_ref;
      if(max_curvature > eps) {
          //max velocity at certain radius
          double max_v = k * (1 / max_curvature);

          v = std::min(v, std::max(v_min, max_v));
      }

      printf("h: %lf, v: %lf\n", horizon_length, v);*/

      double v = v_ref;

      double time = dt;

      for(int i = 0; i < n; i++) {
          double t = pointWithFixedDistance(t_start, v * time);

          path->push_back(PoseStamped({
              spline_x_(t), spline_y_(t), std::atan2(spline_y_.derivative(t), spline_x_.derivative(t)),
              v,
              time
          }));

          time += dt;
      }
  }

  void PathAdapter::LoadReferenceTrajectory(Path ref_traj)
  {
      waypoints_.clear();
      for (int i = 0; i < ref_traj.poses.size(); i++) {
          Point p;
          p.x = ref_traj.poses[i].pose.position.x;
          p.y = ref_traj.poses[i].pose.position.y;
          p.z = ref_traj.poses[i].pose.position.z;
          waypoints_.push_back(p);
      }
      computeSplineWithPoints();

  }

  double PathAdapter::randomFloat() {
    static std::random_device rd;
    static std::mt19937 e(rd());
    static std::uniform_real_distribution<> dist(0, 1);

    return dist(e);
  }

} // namespace Controller