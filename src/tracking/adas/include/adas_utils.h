#ifndef ADAS_UTILS_H
#define ADAS_UTILS_H
#include <iostream>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "control_msgs/msg/control_req.hpp"

// 自然参数
#define R0 6378137.0
#define e 0.0818191908425
const double torad = M_PI / 180;

// 运行频率
#define FRE 100
// 地图路径
//#define DATA_PATH "/SJTU/ros2try/src/tracking/map"
#define DATA_NAME_REAL_ROUTE "/20230215.bin"

// no longer needed
// #define DATA_PATH "/tongji/ros2_tracking/src/tracking/map"


// 地图点格式
struct positionConf
{
  double gpstime;
  double x;
  double y;
  double z;
  double longitude;
  double latitude;
  double height;
  double heading;
  double pitch;
  double roll;
  double velocity;
  double dist;
};

struct PIDConf
{
  double kp;
  double ki;
  double kd;
  double kaw;
  double integrator_saturation_level;
  double output_saturation_level;
  bool integrator_enabled;
};

class PIDControl : public rclcpp::Node
{
public:
  explicit PIDControl(const rclcpp::NodeOptions & node_options);
  ~PIDControl();

  void init(const PIDConf &pid_conf);
  void setPID(const PIDConf &pid_conf);
  void reSet();
  virtual double control(const double error, const double dt);
  int integratorSaturationStatus() const;
  bool integratorHold() const;
  void setIntegratorHold(bool hold);

protected:
  double kp_;
  double ki_;
  double kd_;
  double kaw_;
  double previous_error_;
  double previous_output_;
  double integral_;
  double integrator_saturation_high_;
  double integrator_saturation_low_;
  bool first_hit_;
  bool integrator_enabled_;
  bool integrator_hold_;
  int integrator_saturation_status_;
  double output_saturation_high_;
  double output_saturation_low_;
  int output_saturation_status_;
};

// 地图点的拷贝 p2 = p1
void positionConf_copy(const positionConf &p1, positionConf &p2);

// 坐标变换函数
// 点,原点经纬度
void gps2xy(positionConf &p, const double &longitude, const double &latitude, const double &height);

// 惯导坐标系->车辆中心坐标系
void insgps2center(positionConf &p, const double insgps_x, const double insgps_y);

#endif
