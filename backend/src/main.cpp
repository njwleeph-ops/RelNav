/**
 * REST API Server for RelNav Simulation
 */

#include "cpp-httplib/httplib.h"
#include <nlohmann/json.hpp>

#include "cw_dynamics.hpp"
#include "monte_carlo.hpp"
#include "gnc_algorithms.hpp"

using json = nlohmann::json;
using namespace relnav;


// ----------------------------------------------------------------------------
// JSON Serialization Helpers
// ----------------------------------------------------------------------------

json vec3_to_json(const Vec3& v) {
    return {{"x", v(0)}, {"y", v(1)}, {"z", v(2)}};
}


json vec6_to_json(const Vec6& v) {
    return {
        {"x", v(0)}, {"y", v(1)}, {"z", v(2)}, {"vx", v(3)}, {"vy", v(4)}, {"vz", v(5)}
    };
}


Vec6 json_to_vec6(const json& j) {
    Vec6 v;
    v << j["x"].get<double>(), j["y"].get<double>(), j["z"].get<double>(),
        j["vx"].get<double>(), j["vy"].get<double>(), j["vz"].get<double>();

    return v;
}


Vec3 json_to_vec3(const json& j) {
    Vec3 v;
    v << j["x"].get<double>(), j["y"].get<double>(), j["z"].get<double>();

    return v;
}


json trajectory_to_json(const Trajectory& traj) {
    json result = json::array();
    
    for (int i = 0; i < traj.count; ++i) {
        const auto& pt = traj.points[i];
        result.push_back({{"t", pt.t},
                          {"x", pt.x(0)},
                          {"y", pt.x(1)},
                          {"z", pt.x(2)},
                          {"vx", pt.x(3)},
                          {"vy", pt.x(4)},
                          {"vz", pt.x(5)}
        });
    }

    return result;
}


// ----------------------------------------------------------------------------
// API Helpers
// ----------------------------------------------------------------------------

/// @brief POST "/api/aproach-guidance"
void handle_approach_guidance(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double altitude = body.value("altitude", 420e3);

        // Approach parameters
        ApproachParams params;

        if (body.contains("corridor")) {
            auto& c = body["corridor"];

            if (c.contains("axis")) {
                params.corridor_params.approach_axis = json_to_vec3(c["axis"]);
            }

            params.corridor_params.corridor_angle = c.value("corridorAngle", 0.175);
            params.corridor_params.k = c.value("glideslopeK", 0.001);
            params.corridor_params.min_range = c.value("minRange", 10.0);
        }

        if (body.contains("Q")) {
            auto& Qj = body["Q"];
            double q_pos = Qj.value("pos", 10.0);
            double q_vel = Qj.value("vel", 50.0);
            params.Q = Mat6::Identity();
            params.Q.block<3, 3>(0, 0) *= q_pos;
            params.Q.block<3, 3>(3, 3) *= q_vel;
        }

        if (body.contains("R")) {
            double r = body["R"].get<double>();
            params.R = Mat3::Identity() * r;
        }

        params.u_max = body.value("uMax", 0.01);
        params.dt = body.value("dt", 1.0);
        params.timeout = body.value("timeout", 6000.0);
        params.success_range = body.value("successRange", 5.0);
        params.success_velocity = body.value("successVelocity", 0.05);

        OrbitalParams orbit(altitude);
        auto result = run_approach_guidance(x0, orbit.mean_motion(), params);

        // Build control and waypoint histories
        json control_history = json::array();
        json waypoint_history = json::array();

        for (int i = 0; i < result.num_points - 1; ++i) {
            control_history.push_back(vec3_to_json(result.control_history[i]));
            waypoint_history.push_back(vec3_to_json(result.waypoint_history[i]));
        }

        json response = {
            {"trajectory", trajectory_to_json(result.trajectory)},
            {"controlHistory", control_history},
            {"waypointHistory", waypoint_history},
            {"success", result.success},
            {"totalDV", result.total_dv},
            {"finalRange", result.final_range},
            {"finalVelocity", result.final_velocity},
            {"duration", result.duration},
            {"saturationCount", result.saturation_count}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief POST "/api/monte-carlo"
void handle_monte_carlo(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double altitude = body.value("altitude", 420e3);
        int n_samples = body.value("nSamples", 1000);
        int n_threads = body.value("nThreads", 0);
        unsigned int seed = body.value("seed", 42);

        // Approach parameters
        ApproachParams params;

        if (body.contains("corridor"))
        {
            auto &c = body["corridor"];

            if (c.contains("axis"))
            {
                params.corridor_params.approach_axis = json_to_vec3(c["axis"]);
            }

            params.corridor_params.corridor_angle = c.value("corridorAngle", 0.175);
            params.corridor_params.k = c.value("glideslopeK", 0.001);
            params.corridor_params.min_range = c.value("minRange", 10.0);
        }

        if (body.contains("Q"))
        {
            auto &Qj = body["Q"];
            double q_pos = Qj.value("pos", 10.0);
            double q_vel = Qj.value("vel", 50.0);
            params.Q = Mat6::Identity();
            params.Q.block<3, 3>(0, 0) *= q_pos;
            params.Q.block<3, 3>(3, 3) *= q_vel;
        }

        if (body.contains("R"))
        {
            double r = body["R"].get<double>();
            params.R = Mat3::Identity() * r;
        }

        params.u_max = body.value("uMax", 0.01);
        params.dt = body.value("dt", 1.0);
        params.timeout = body.value("timeout", 6000.0);
        params.success_range = body.value("successRange", 5.0);
        params.success_velocity = body.value("successVelocity", 0.05);

        // Uncertainty model
        UncertaintyModel uncertainty;

        if (body.contains("uncertainty")) {
            auto& u = body["uncertainty"];

            if (u.contains("posError")) {
                uncertainty.pos_error = json_to_vec3(u["posError"]);
            }

            if (u.contains("velError")) {
                uncertainty.vel_error = json_to_vec3(u["velError"]);
            }

            if (u.contains("thrustMagError")) {
                uncertainty.thrust_mag_error = u["thrustMagError"].get<double>();
            }

            if (u.contains("thrustPointingError")) {
                uncertainty.thrust_pointing_error = u["thrustPointingError"].get<double>();
            }
        }

        OrbitalParams orbit(altitude);
        auto result = run_monte_carlo(x0, orbit.mean_motion(), params,
                                      uncertainty, n_samples, n_threads, seed);
        
        // Build sample results
        json samples = json::array();
        int limit = std::min(result.n_samples, 500);

        for (int i = 0; i < limit; ++i) {
            samples.push_back({{"success", result.samples[i].success},
                               {"finalRange", result.samples[i].final_range},
                               {"finalVelocity", result.samples[i].final_velocity},
                               {"totalDV", result.samples[i].total_dv},
                               {"duration", result.samples[i].duration},
                               {"saturationCount", result.samples[i].saturation_count}
            });
        }

        // Build final states array
        json final_states = json::array();

        for (int i = 0; i < limit; ++i) {
            final_states.push_back(vec6_to_json(result.final_states[i]));
        }

        json response = {
            {"nSamples", result.n_samples},
            {"nSuccess", result.n_success},
            {"successRate", result.success_rate},
            {"meanDV", result.mean_dv},
            {"stdDV", result.std_dv},
            {"meanDuration", result.mean_duration},
            {"stdDuration", result.std_duration},
            {"meanFinalRange", result.mean_final_range},
            {"meanSaturationCount", result.mean_saturation_count},
            {"samples", samples},
            {"finalStates", final_states}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief POST "/api/propagate"
void handle_propagate(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double duration = body.value("duration", 5569.0);
        double altitude = body.value("altitude", 420e3);
        int num_points = body.value("numPoints", 500);

        OrbitalParams orbit(altitude);
        Trajectory traj;
        propagate_analytical(x0, duration, orbit.mean_motion(), traj, num_points);

        json response = {
            {"trajectory", trajectory_to_json(traj)},
            {"orbit", {{"altitude", altitude}, {"meanMotion", orbit.mean_motion()}, {"period", orbit.period()}}}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief POST "/api/validate"
void handle_validate(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double duration = body.value("duration", 11138.0);
        double altitude = body.value("altitude", 420e3);
        int num_steps = body.value("numSteps", 1000);

        OrbitalParams orbit(altitude);
        auto result = validate_propagators(x0, duration, orbit.mean_motion(), num_steps);

        json response = {
            {"maxPositionError", result.max_pos_err},
            {"maxVelocityError", result.max_vel_err},
            {"maxRelativePositionError", result.max_rel_pos_err},
            {"passed", result.passed}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief GET "/api/orbit"
void handle_orbit_info(const httplib::Request& req, httplib::Response& res) {
    try {
        double altitude = 420e3;

        if (req.has_param("altitude")) {
            altitude = std::stod(req.get_param_value("altitude"));
        }

        OrbitalParams orbit(altitude);

        json response = {
            {"altitude", altitude},
            {"radius", orbit.radius()},
            {"meanMotion", orbit.mean_motion()},
            {"period", orbit.period()},
            {"periodMinutes", orbit.period() / 60.0}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    httplib::Server svr;

    int port = 8080;

    if (argc > 1) {
        port = std::stoi(argv[1]);
    }

    // CORS middleware
    svr.set_pre_routing_handler([](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");

        if (req.method == "OPTIONS") {
            res.status = 204;
            return httplib::Server::HandlerResponse::Handled;
        }

        return httplib::Server::HandlerResponse::Unhandled;
    });

    // Register API's
    svr.Get("/api/health", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content(json({{"status", "ok"}, {"version", "2.0.0"}}).dump(),
                        "application/json");
    });

    svr.Get("/api/orbit", handle_orbit_info);
    svr.Post("/api/propagate", handle_propagate);
    svr.Post("/api/validate", handle_validate);
    svr.Post("/api/approach-guidance", handle_approach_guidance);
    svr.Post("/api/monte-carlo", handle_monte_carlo);

    std::cout << "RelNav-MC API Server v2.0 starting on port " << port << std::endl;
    std::cout << "Endpoints:" << std::endl;
    std::cout << "  GET  /api/health" << std::endl;
    std::cout << "  GET  /api/orbit?altitude=<m>" << std::endl;
    std::cout << "  POST /api/propagate" << std::endl;
    std::cout << "  POST /api/validate" << std::endl;
    std::cout << "  POST /api/approach-guidance" << std::endl;
    std::cout << "  POST /api/monte-carlo" << std::endl;

    svr.listen("0.0.0.0", port);

    return 0;
}