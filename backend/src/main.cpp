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


json stateHistory_to_json(const StateHistory& history) {
    json result = json::array();
    
    for (const auto& [t, x] : history) {
        result.push_back({{"t", t},
                          {"x", x(0)},
                          {"y", x(1)},
                          {"z", x(2)},
                          {"vx", x(3)},
                          {"vy", x(4)},
                          {"vz", x(5)}
                        });
    }

    return result;
}


// ----------------------------------------------------------------------------
// API Helpers
// ----------------------------------------------------------------------------

/// @brief POST "/api/propagate"
void handle_propagate(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double duration = body.value("duration", 5569.0);    // Default: 1 orbit
        double altitude = body.value("altitude", 420e3);     // Default: ISS
        std::string method = body.value("method", "analytical");
        int num_points = body.value("numPoints", 500);

        OrbitalParams orbit(altitude);
        double n = orbit.mean_motion();

        StateHistory history;
        if (method == "analytical") {
            history = propagate_analytical(x0, duration, n, num_points);
        } else {
            history = propagate_numerical(x0, duration, n, num_points);
        }

        json response = {
            {"trajectory", stateHistory_to_json(history)},
            {"orbit", {{"altitude", altitude}, {"meanMotion", n}, {"period", orbit.period()}}} 
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
        double duration = body.value("duration", 11138.0);  // Default: 2 orbits
        double altitude = body.value("altitude", 420e3);    // Default: ISS

        OrbitalParams orbit(altitude);
        auto result = validate_propagators(x0, duration, orbit.mean_motion());

        json response = {
            {"maxPositionError", result.max_pos_error},
            {"maxVelocityError", result.max_vel_error},
            {"maxRelativePositionError", result.max_rel_pos_error},
            {"passed", result.passed}
        };
        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}


/// @brief POST "/api/targeted-monte-carlo"
void handle_targeted_monte_carlo(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        Vec3 target = json_to_vec3(body["target"]);
        double tof = body["timeOfFlight"].get<double>();
        double altitude = body.value("altitude", 420e3);
        int n_samples = body.value("nSamples", 1000);
        unsigned int seed = body.value("seed", 42);

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

        // Corridor
        ApproachCorridor corridor;
        
        if (body.contains("corridor")) {
            auto& c = body["corridor"];
            std::string type = c.value("type", "cylinder");

            if (type == "cone") {
                corridor.type = ApproachCorridor::Type::Cone;
            } else if (type == "box") {
                corridor.type = ApproachCorridor::Type::Box;
            }

            corridor.radius = c.value("radius", 200.0);
            corridor.cone_half_angle = c.value("coneHalfAngle", 0.175);
            
            if (c.contains("boxHalfWidth")) {
                corridor.box_half_width = json_to_vec3(c["boxHalfWidth"]);
            }
        }

        OrbitalParams orbit(altitude);
        auto result = run_targeted_monte_carlo(x0, target, tof, orbit.mean_motion(),
                                               uncertainty, corridor, n_samples, seed);

        // Build final postions array
        json final_positions = json::array();
        json inside_flags = json::array();
        int limit = std::min(n_samples, 500);

        for (int i = 0; i < limit; ++i) {
            Vec6 xf = result.final_states[i];
            final_positions.push_back({{"x", xf(0)}, {"y", xf(1)}, {"z", xf(2)}});
            inside_flags.push_back(result.inside_corridor[i]);
        }

        json response = {
            {"target", vec3_to_json(result.target)},
            {"nominalDV", {{"dv1", vec3_to_json(result.nominal_maneuver.dv1)}, {"dv2", vec3_to_json(result.nominal_maneuver.dv2)}, {"totalDV", result.nominal_maneuver.total_dv}}},
            {"finalPositions", final_positions},
            {"insideCorridor", inside_flags},
            {"meanArrival", vec3_to_json(result.mean_arrival)},
            {"sigma3Arrival", vec3_to_json(result.sigma3_arrival)},
            {"corridorSuccessRate", result.corridor_success_rate},
            {"nSamples", result.n_samples}
        };
        res.set_content(response.dump(), "application/json");   
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}


/// @brief POST "/api/lqr-approach"
void handle_lqr_approach(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        double duration = body.value("duration", 2784.7);       // Default: Half orbit
        double altitude = body.value("altitude", 420e3);        // Default: ISS
        double u_max = body.value("uMax", 0.01);
        double dt = body.value("dt", 10.0);

        // LQR weights
        Mat6 Q = Mat6::Identity();
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

        if (body.contains("Q")) {
            auto& Qj = body["Q"];
            Q(0, 0) = Qj.value("pos", 1.0);
            Q(1, 1) = Qj.value("pos", 1.0);
            Q(2, 2) = Qj.value("pos", 1.0);
            Q(3, 3) = Qj.value("vel", 0.1);
            Q(4, 4) = Qj.value("vel", 0.1);
            Q(5, 5) = Qj.value("vel", 0.1);
        }

        if (body.contains("R")) {
            double r = body["R"].get<double>();
            R = Eigen::Matrix3d::Identity() * r;
        }

        OrbitalParams orbit(altitude);
        auto result = simulate_lqr_approach(x0, duration, orbit.mean_motion(), Q, R, u_max, dt);

        // Build response
        json control_history = json::array();
        for (const auto& u : result.control_history) {
            control_history.push_back(vec3_to_json(u));
        }

        json response = {
            {"trajectory", stateHistory_to_json(result.trajectory)},
            {"controlHistory", control_history},
            {"totalDV", result.total_dv}
        };
        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}


/// @brief POST "/api/glideslope-check"
void handle_glideslope_check(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x = json_to_vec6(body["state"]);

        GlideslopeParams params;

        if (body.contains("k")) {
            params.k = body["k"].get<double>();
        }

        auto result = check_glideslope_violation(x, params);

        json response = {
            {"violated", result.violated},
            {"margin", result.margin},
            {"approachVelocity", result.v_approach},
            {"maxVelocity", result.v_max}
        };
        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}


/// @brief POST "/api/dv-sweep"
void handle_dv_sweep(const httplib::Request &req, httplib::Response &res)
{
    try
    {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        Vec3 rf = json_to_vec3(body["targetPosition"]);
        double tof_min = body.value("tofMin", 300.0);
        double tof_max = body.value("tofMax", 5569.0);
        int num_points = body.value("numPoints", 50);
        double altitude = body.value("altitude", 420e3);

        OrbitalParams orbit(altitude);
        double n = orbit.mean_motion();

        Vec3 r0 = x0.head<3>();
        Vec3 v0 = x0.tail<3>();

        json results = json::array();
        double dt = (tof_max - tof_min) / (num_points - 1);

        for (int i = 0; i < num_points; ++i)
        {
            double tof = tof_min + i * dt;
            auto maneuver = two_impulse_targeting(r0, v0, rf, tof, n);

            results.push_back({{"tof", tof},
                               {"dv1", vec3_to_json(maneuver.dv1)},
                               {"dv2", vec3_to_json(maneuver.dv2)},
                               {"totalDV", maneuver.total_dv}});
        }

        json response = {{"sweep", results}};
        res.set_content(response.dump(), "application/json");
    }
    catch (const std::exception &e)
    {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}


/// @brief GET "/api/orbit"
void handle_orbit_info(const httplib::Request& req, httplib::Response& res) {
    try {
        double altitude = 420e3;    // Default: ISS

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

    svr.Get("/api/health", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content(json({{"status", "ok"}, {"version", "1.0.0"}}).dump(), "application/json");
    });
    svr.Get("/api/orbit", handle_orbit_info);
    svr.Post("/api/propagate", handle_propagate);
    svr.Post("/api/validate", handle_validate);
    svr.Post("/api/targeted-monte-carlo", handle_targeted_monte_carlo);
    svr.Post("/api/two-impulse", handle_two_impulse);
    svr.Post("/api/lqr-approach", handle_lqr_approach);
    svr.Post("/api/glideslope-check", handle_glideslope_check);
    svr.Post("/api/dv-sweep", handle_dv_sweep);

    std::cout << "RelNav-MC API Server starting on port " << port << std::endl;
    std::cout << "Endpoints:" << std::endl;
    std::cout << "  GET  /api/health" << std::endl;
    std::cout << "  GET  /api/orbit?altitude=<m>" << std::endl;
    std::cout << "  POST /api/propagate" << std::endl;
    std::cout << "  POST /api/validate" << std::endl;
    std::cout << "  POST /api/targeted-monte-carlo" << std::endl;
    std::cout << "  POST /api/two-impulse" << std::endl;
    std::cout << "  POST /api/lqr-approach" << std::endl;
    std::cout << "  POST /api/glideslope-check" << std::endl;
    std::cout << "  POST /api/dv-sweep" << std::endl;

    svr.listen("0.0.0.0", port);

    return 0;
}