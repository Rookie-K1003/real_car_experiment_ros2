#pragma once

#include <tuple>
#include <cmath>

class LlaToMapConverter {
public:
    LlaToMapConverter(double ref_lat_deg, double ref_lon_deg, double ref_alt_m);

    std::tuple<double, double, double> convert(double lat_deg, double lon_deg, double alt_m) const;

private:
    static constexpr double a = 6378137.0;                      // WGS84 椭球体长半轴
    static constexpr double f = 1.0 / 298.257223563;            // 扁率
    static constexpr double e2 = f * (2 - f);                   // 第一偏心率平方

    double ref_lat_rad_;
    double ref_lon_rad_;
    double ref_ecef_x_;
    double ref_ecef_y_;
    double ref_ecef_z_;

    std::tuple<double, double, double> lla2ecef(double lat_deg, double lon_deg, double alt) const;
    std::tuple<double, double, double> ecef2enu(double x, double y, double z) const;
};