/**
 * @file config.cpp
 * @brief Function definitions for configuring GNC controller parameters
 */

#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include "config.hpp"

namespace relnav {
    
GNCConfig load_config(const std::string& filepath) {
    GNCConfig config;

    YAML::Node yaml;

    try {
        yaml = YAML::LoadFile(filepath);
    } catch (const YAML::BadFile& e) {
        throw std::runtime_error("Failed to open config: " + filepath);
    }

    if (yaml["orbit"]) {
        config.altitude = yaml["orbit"]["altitude"].as<double>(420e3);
    }

    if (yaml["lqr"]) {
        config.Q_pos_along = yaml["lqr"]["Q_pos_along"].as<double>(1.0);
        config.Q_pos_cross = yaml["lqr"]["Q_pos_cross"].as<double>(50.0);
        config.Q_vel = yaml["lqr"]["Q_vel"].as<double>(0.1);
        config.R_val = yaml["lqr"]["R"].as<double>(1.0);
    }

    if (yaml["glideslope"]) {
        config.glideslope_k = yaml["glideslope"]["k"].as<double>(0.001);
        config.corridor_angle_rad = yaml["glideslope"]["corridor_angle"].as<double>(0.0175);
        config.min_range = yaml["glideslope"]["min_range"].as<double>(5.0);

        if (yaml["glideslope"]["approach_axis"]) {
            auto axis = yaml["glideslope"]["approach_axis"];
            config.approach_axis = Vec3{
                axis[0].as<double>(),
                axis[1].as<double>(),
                axis[2].as<double>()
            };
        }
    }

    if (yaml["nav_filter"]) {
        auto nf = yaml["nav_filter"];

        if (nf["sensor"]) {
            config.sensor.range_noise = nf["sensor"]["range_noise"].as<double>(1.0);
            config.sensor.bearing_noise = nf["sensor"]["bearing_noise"].as<double>(0.005);
            config.sensor.update_rate = nf["sensor"]["update_rate"].as<double>(1.0);
            config.sensor.max_range = nf["sensor"]["max_range"].as<double>(2000.0);
            config.sensor.min_range = nf["sensor"]["min_range"].as<double>(2.0);
        }

        config.Q_process_pos = nf["Q_process_pos"].as<double>(0.001);
        config.Q_process_vel = nf["Q_process_vel"].as<double>(0.0001);
        config.R_meas_range = nf["R_meas_range"].as<double>(1.0);
        config.R_meas_bearing = nf["R_meas_bearing"].as<double>(0.000025);
        config.P0_pos = nf["P0_pos"].as<double>(100.0);
        config.P0_vel = nf["P0_vel"].as<double>(0.01);
    }

    if (yaml["control"]) {
        config.u_max = yaml["control"]["u_max"].as<double>(0.01);
        config.thrust_mag_sigma = yaml["control"]["thrust_mag_sigma"].as<double>(0.01);
        config.thrust_pointing_sigma = yaml["control"]["thrust_pointing_sigma"].as<double>(0.005);
    }

    if (yaml["sim"]) {
        config.dt = yaml["sim"]["dt"].as<double>(1.0);
        config.timeout = yaml["sim"]["timeout"].as<double>(5000.0);
        config.success_range = yaml["sim"]["success_range"].as<double>(5.0);
        config.success_velocity = yaml["sim"]["success_velocity"].as<double>(0.05);
    }

    if (yaml["mc"]) {
        config.n_threads = yaml["mc"]["n_threads"].as<int>(0);
        config.seed = yaml["mc"]["seed"].as<unsigned int>(42);
    }

    std::cout << "Config loaded from " << filepath << std::endl;

    return config;
}

ApproachParams GNCConfig::to_approach_params() const {
    ApproachParams params;

    Vec3 axis = approach_axis.normalized().cwiseAbs();

    params.Q = Mat6::Identity();

    for (int i = 0; i < 3; ++i) {
        if (axis(i) > 0.9) {
            params.Q(i, i) = Q_pos_along;
        } else {
            params.Q(i, i) = Q_pos_cross;
        }
    }

    params.Q.block<3, 3>(3, 3) *= Q_vel;
    params.R = Mat3::Identity() * R_val;

    params.corridor_params.k = glideslope_k;
    params.corridor_params.corridor_angle = corridor_angle_rad;
    params.corridor_params.min_range = min_range;
    params.corridor_params.approach_axis = approach_axis;

    params.u_max = u_max;
    params.dt = dt;
    params.timeout = timeout;
    params.success_range = success_range;
    params.success_velocity = success_velocity;

    return params;
}

NavConfig GNCConfig::to_nav_config() const {
    NavConfig nav_config;
    
    nav_config.sensor = sensor;
    nav_config.filter.Q_process = Mat6::Identity();
    nav_config.filter.Q_process.block<3, 3>(0, 0) *= Q_process_pos;
    nav_config.filter.Q_process.block<3, 3>(3, 3) *= Q_process_vel;

    nav_config.filter.R_meas = Mat3::Identity();
    nav_config.filter.R_meas(0, 0) *= R_meas_range;
    nav_config.filter.R_meas.block<2, 2>(1, 1) *= R_meas_bearing;

    nav_config.filter.P0 = Mat6::Identity();
    nav_config.filter.P0.block<3, 3>(0, 0) *= P0_pos;
    nav_config.filter.P0.block<3, 3>(3, 3) *= P0_vel;

    return nav_config;
}

UncertaintyModel GNCConfig::to_uncertainty() const {
    UncertaintyModel uncertainty;

    uncertainty.thrust_mag_error = thrust_mag_sigma;
    uncertainty.thrust_pointing_error = thrust_pointing_sigma;

    return uncertainty;
}

}