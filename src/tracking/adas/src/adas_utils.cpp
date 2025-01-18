#include "adas_utils.h"

PIDControl::PIDControl(const rclcpp::NodeOptions & node_options) : Node("adas_utils", node_options)
{
  RCLCPP_INFO(get_logger(), "PIDControl constructor Fun.");
}

PIDControl::~PIDControl()
{
  RCLCPP_INFO(get_logger(), "PIDControl destructor Fun.");
}

void PIDControl::init(const PIDConf &pid_conf)
{
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = false;
  integrator_enabled_ = false;
  integrator_hold_ = false;

  integrator_saturation_high_ = 0.0;
  integrator_saturation_low_ = 0.0;
  integrator_saturation_status_ = 0;

  setPID(pid_conf);
}

void PIDControl::setPID(const PIDConf &pid_conf)
{
  kp_ = pid_conf.kp;
  ki_ = pid_conf.ki;
  kd_ = pid_conf.kd;
  kaw_ = pid_conf.kaw;
  RCLCPP_INFO(get_logger(), "kp:%f,ki:%f,kd:%f", kp_, ki_, kd_);
}

void PIDControl::reSet()
{
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
}

double PIDControl::control(const double error, const double dt)
{
  if (dt <= 0)
  {
    RCLCPP_INFO(get_logger(), "dt:%lf <= 0, control = %lf", dt, previous_output_);
    return previous_output_;
  }

  double ans_PID = 0;
  double diff = 0;

  if (first_hit_)
  {
    first_hit_ = false;
  }
  else
  {
    diff = error - previous_error_;
  }

  if (!integrator_enabled_)
  {
    integral_ = 0;
  }
  else
  {
    integral_ += error * dt * ki_;
    if (!integrator_hold_)
    {
      if (integral_ > integrator_saturation_high_)
      {
        integral_ = integrator_saturation_high_;
        integrator_saturation_status_ = 1;
      }
      else if (integral_ < integrator_saturation_low_)
      {
        integral_ = integrator_saturation_low_;
        integrator_saturation_status_ = -1;
      }
      else
      {
        integrator_saturation_status_ = 0;
      }
    }
  }

  ans_PID = error * kp_ + integral_ + diff * kd_;
  //ans_PID = 0.6*previous_output_ + 0.4*ans_PID;
  //RCLCPP_INFO(get_logger(), "PID ANS:%f,kp_%f %f, %f",ans_PID,kp_,error,previous_error_);
  previous_error_ = error;
  previous_output_ = ans_PID;
  return ans_PID;
}

int PIDControl::integratorSaturationStatus() const
{
  return integrator_saturation_status_;
}

bool PIDControl::integratorHold() const
{
  return integrator_hold_;
}
void PIDControl::setIntegratorHold(bool hold)
{
  integrator_hold_ = hold;
}

// 地图点的拷贝 p2 = p1
void positionConf_copy(const positionConf &p1, positionConf &p2)
{
    p2.gpstime = p1.gpstime;
    p2.x = p1.x;
    p2.y = p1.y;
    p2.z = p1.z;
    p2.longitude = p1.longitude;
    p2.latitude = p1.latitude;
    p2.height = p1.height;
    p2.heading = p1.heading;
    p2.pitch = p1.pitch;
    p2.roll = p1.roll;
    p2.velocity = p1.velocity;
    p2.dist = p1.dist;
}

// 坐标变换函数
// 点,原点经纬度
void gps2xy(positionConf &p, const double &longitude, const double &latitude, const double &height)
{
    double Re = R0 / sqrt(1 - e * e * sin(latitude * torad) * sin(latitude * torad));
    double x0_ECEF = (Re + height) * cos(latitude * torad) * cos(longitude * torad);
    double y0_ECEF = (Re + height) * cos(latitude * torad) * sin(longitude * torad);
    double z0_ECEF = (Re * (1 - e * e) + height) * sin(latitude * torad);

    Re = R0 / sqrt(1 - e * e * sin(p.latitude * torad) * sin(p.latitude * torad));
    double dx_ECEF = (Re + p.height) * cos(p.latitude * torad) * cos(p.longitude * torad) - x0_ECEF;
    double dy_ECEF = (Re + p.height) * cos(p.latitude * torad) * sin(p.longitude * torad) - y0_ECEF;
    double dz_ECEF = (Re * (1 - e * e) + p.height) * sin(p.latitude * torad) - z0_ECEF;

    // ECEF to ENU
    p.x = -sin(p.longitude * torad) * dx_ECEF + cos(p.longitude * torad) * dy_ECEF;
    p.y = -sin(p.latitude * torad) * cos(p.longitude * torad) * dx_ECEF - sin(p.latitude * torad) * sin(p.longitude * torad) * dy_ECEF + cos(p.latitude * torad) * dz_ECEF;
    p.z = cos(p.latitude * torad) * cos(p.longitude * torad) * dx_ECEF + cos(p.latitude * torad) * sin(p.longitude * torad) * dy_ECEF + sin(p.latitude * torad) * dz_ECEF;
}

// 惯导坐标系->车辆中心坐标系
void insgps2center(positionConf &p, const double insgps_x, const double insgps_y)
{
    p.x = p.x - (insgps_x * cos(p.heading * torad) + insgps_y * sin(p.heading * torad));
    p.y = p.y - (insgps_y * cos(p.heading * torad) - insgps_x * sin(p.heading * torad));
}