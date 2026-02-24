/**
 * @file config.hpp
 * @brief Header for configuration file 
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

#include "cw_dynamics.hpp"
#include "gnc_algorithms.hpp"
#include "monte_carlo.hpp"

namespace relnav {
    
struct GNCConfig {
    double altitude = 420e3;

    double Q_pos_along = 1.0;
    double Q_pos_cross = 50.0;
    double Q_vel = 0.1;
    double R_val = 1.0;

    double glideslope_k = 0.001;
    double corridor_angle_rad = 0.0175;
    double min_range = 5.0;
    Vec3 approach_axis = Vec3{0, -1, 0};

    SensorModel sensor;
    double Q_process_pos = 0.001;
    double Q_process_vel = 0.0001;
    double R_meas_range = 1.0;
    double R_meas_bearing = 0.000025;
    double P0_pos = 100.0;
    double P0_vel = 0.01;

    double u_max = 0.01;
    double thrust_mag_sigma = 0.01;
    double thrust_pointing_sigma = 0.005;

    double dt = 1.0;
    double timeout = 5000.0;
    double success_range = 5.0;
    double success_velocity = 0.05;

    int n_threads = 0;
    unsigned int seed = 42;

    ApproachParams to_approach_params() const;
    NavConfig to_nav_config() const;
    UncertaintyModel to_uncertainty() const;
};

GNCConfig load_config(const std::string& filepath);
}

#endif 
