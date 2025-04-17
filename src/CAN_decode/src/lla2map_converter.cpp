#include "lla2map_converter.hpp"

LlaToMapConverter::LlaToMapConverter(double ref_lat_deg, double ref_lon_deg, double ref_alt_m) {
    ref_lat_rad_ = ref_lat_deg * M_PI / 180.0;
    ref_lon_rad_ = ref_lon_deg * M_PI / 180.0;
    std::tie(ref_ecef_x_, ref_ecef_y_, ref_ecef_z_) = lla2ecef(ref_lat_deg, ref_lon_deg, ref_alt_m);
}

std::tuple<double, double, double> LlaToMapConverter::convert(double lat_deg, double lon_deg, double alt_m) const {
    auto [x, y, z] = lla2ecef(lat_deg, lon_deg, alt_m);
    return ecef2enu(x, y, z);
}

std::tuple<double, double, double> LlaToMapConverter::lla2ecef(double lat_deg, double lon_deg, double alt) const {
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    double sin_lat = sin(lat), cos_lat = cos(lat);
    double sin_lon = sin(lon), cos_lon = cos(lon);

    double N = a / sqrt(1 - e2 * sin_lat * sin_lat);

    double x = (N + alt) * cos_lat * cos_lon;
    double y = (N + alt) * cos_lat * sin_lon;
    double z = (N * (1 - e2) + alt) * sin_lat;

    return {x, y, z};
}

std::tuple<double, double, double> LlaToMapConverter::ecef2enu(double x, double y, double z) const {
    double dx = x - ref_ecef_x_;
    double dy = y - ref_ecef_y_;
    double dz = z - ref_ecef_z_;

    double sin_lat = sin(ref_lat_rad_), cos_lat = cos(ref_lat_rad_);
    double sin_lon = sin(ref_lon_rad_), cos_lon = cos(ref_lon_rad_);

    double xEast  = -sin_lon * dx + cos_lon * dy;
    double yNorth = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    double zUp    =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    return {xEast, yNorth, zUp};
}