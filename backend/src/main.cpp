/**
 * REST API Server for RelNav Simulation
 */

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <array>

#include "cpp-httplib/httplib.h"
#include <nlohmann/json.hpp>

#include "cw_dynamics.hpp"
#include "monte_carlo.hpp"
#include "gnc_algorithms.hpp"
#include "config.hpp"

using json = nlohmann::json;
using namespace relnav;

/// Constants for config file
static GNCConfig g_config;
static std::mutex g_config_mutex;

// ----------------------------------------------------------------------------
// JSON Serialization Helpers
// ----------------------------------------------------------------------------

static json vec3_to_json(const Vec3& v) {
    return {{"x", v(0)}, {"y", v(1)}, {"z", v(2)}};
}


static json vec6_to_json(const Vec6& v) {
    return {
        {"x", v(0)}, {"y", v(1)}, {"z", v(2)}, {"vx", v(3)}, {"vy", v(4)}, {"vz", v(5)}
    };
}


static Vec6 json_to_vec6(const json& j) {
    Vec6 v;
    v << j["x"].get<double>(), j["y"].get<double>(), j["z"].get<double>(),
        j["vx"].get<double>(), j["vy"].get<double>(), j["vz"].get<double>();

    return v;
}


static Vec3 json_to_vec3(const json& j) {
    Vec3 v;
    v << j["x"].get<double>(), j["y"].get<double>(), j["z"].get<double>();

    return v;
}

static json trajectory_to_json(const Trajectory& traj) {
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

static json percentile_stats_to_json(const PercentileStats& percentile) {
    return {
        {"min", percentile.min},
        {"max", percentile.max},
        {"p50", percentile.p50},
        {"p75", percentile.p75},
        {"p90", percentile.p90},
        {"p95", percentile.p95},
        {"p99", percentile.p99}
    };
}

static json failure_breakdown_to_json(const FailureBreakdown& failures) {
    return {
        {"positionTimeout", failures.position_timeout},
        {"excessVelocity", failures.excess_velocity}
    };
}

static EnvelopeConfig parse_envelope_config(const json& body) {
    EnvelopeConfig config;

    config.range_min = body.value("/sweep/rangeMin"_json_pointer, 100.0);
    config.range_max = body.value("/sweep/rangeMax"_json_pointer, 3000.0);
    config.range_steps = body.value("/sweep/rangeSteps"_json_pointer, 20);
    config.u_max_min = body.value("/sweep/uMaxMin"_json_pointer, 0.001);
    config.u_max_max = body.value("/sweep/uMaxMax"_json_pointer, 0.05);
    config.u_max_steps = body.value("/sweep/uMaxSteps"_json_pointer, 15);
    config.samples_per_point = body.value("/sweep/samplesPerPoint", 200);

    return config;
}

// ----------------------------------------------------------------------------
// Approach Corridor Visualization
// ----------------------------------------------------------------------------

struct ConeMesh {
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<int, 3>> faces;
};

ConeMesh generate_corridor_cone(
    const Vec3& approach_axis,
    double corridor_angle,
    double max_range,
    int segments = 32) 
{
    ConeMesh mesh;

    Vec3 axis = approach_axis.normalized();
    
    Vec3 perpendicular_1 = axis.cross(Vec3::UnitX());

    if (perpendicular_1.norm() < 1e-6) {
        Vec3 perpendicular_1 = axis.cross(Vec3::UnitZ());
    }

    perpendicular_1.normalize();
    Vec3 perpendicular_2 = axis.cross(perpendicular_1).normalized();

    double r_end = max_range * std::tan(corridor_angle);

    std::array<double, 3> origin{0.0, 0.0, 0.0};

    mesh.vertices.push_back(origin);

    for (int i = 0; i < segments; ++i) {
        double theta = 2.0 * M_PI * i / segments;
        Vec3 point = axis * max_range
                   + perpendicular_1 * (r_end * std::cos(theta))
                   + perpendicular_2 * (r_end * std::sin(theta));
        mesh.vertices.push_back({point(0), point(1), point(2)});
    }

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        mesh.faces.push_back({0, i + 1, next + 1});
    }

    return mesh;
}

// ----------------------------------------------------------------------------
// Performance Envelope Helpers
// ----------------------------------------------------------------------------

struct EnvelopeJob {
    int id = 0;
    std::string status = "running";
    std::atomic<int> cells_complete{0};
    int total_cells = 0;
    std::string error_msg;

    std::shared_ptr<PerformanceEnvelope> result;
    std::thread worker;

    // Delete only if out of scope
    ~EnvelopeJob() {
        if (worker.joinable()) {
            worker.join();
        }
    }
};

// Global job storage
std::mutex jobs_mutex;
std::unordered_map<int, std::shared_ptr<EnvelopeJob>> envelope_jobs;
std::atomic<int> next_job_id{1};

// ----------------------------------------------------------------------------
// API Handlers
// ----------------------------------------------------------------------------

/// @brief GET "/api/config"
static void handle_get_config(const httplib::Request& req, httplib::Response& res) {
    try {
        std::lock_guard<std::mutex> lock(g_config_mutex);

        json response = {
            {"orbit", {
                {"altitude", g_config.altitude}
            }},
            {"lqr", {
                {"Q_pos_along", g_config.Q_pos_along},
                {"Q_pos_cross", g_config.Q_pos_cross},
                {"Q_vel", g_config.Q_vel},
                {"R", g_config.R_val}
            }},
            {"glideslope", {
                {"k", g_config.glideslope_k},
                {"corridor_angle_rad", g_config.corridor_angle_rad},
                {"min_range", g_config.min_range},
                {"approach_axis", {g_config.approach_axis(0), g_config.approach_axis(1), g_config.approach_axis(2)}}
            }},
            {"nav_filter",{
                {"sensor", {
                    {"range_noise", g_config.sensor.range_noise},
                    {"bearing_noise", g_config.sensor.bearing_noise},
                    {"update_rate", g_config.sensor.update_rate},
                    {"max_range", g_config.sensor.max_range},
                    {"min_range", g_config.sensor.min_range}
                }},
                {"Q_process_pos", g_config.Q_process_pos},
                {"Q_process_vel", g_config.Q_process_vel},
                {"R_meas_range", g_config.R_meas_range},
                {"R_meas_bearing", g_config.R_meas_bearing},
                {"P0_pos", g_config.P0_pos},
                {"P0_vel", g_config.P0_vel}
            }},
            {"control", {
                {"u_max", g_config.u_max},
                {"thrust_mag_sigma", g_config.thrust_mag_sigma},
                {"thrust_pointing_sigma", g_config.thrust_pointing_sigma}
            }},
            {"sim", {
                {"dt", g_config.dt},
                {"timeout", g_config.timeout},
                {"success_range", g_config.success_range},
                {"success_velocity", g_config.success_velocity}
            }},
            {"mc", {
                {"n_threads", g_config.n_threads},
                {"seed", g_config.seed}
            }}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 500;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
}

/// @brief POST "/api/config/reload"
static void handle_reload_config(const httplib::Request& req, httplib::Response& res) {
    try {
        GNCConfig new_config = load_config("gnc_config.yaml");

        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            g_config = new_config;
        }

        res.set_content(
            json({{"status", "ok"}, {"message", "Config reloaded"}}).dump(),
            "application/json"
        );
    } catch (const std::exception& e) {
        res.status = 500;
        res.set_content(
            json({{"error", e.what()}}).dump(),
            "application/json"
        );
    }
}

/// @brief POST "/api/approach-guidance"
static void handle_approach_guidance(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);

        ApproachParams params;
        NavConfig nav;
        
        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            params = g_config.to_approach_params();
            nav = g_config.to_nav_config();
        }
        
        double altitude = g_config.altitude;
        double n = OrbitalParams(altitude).mean_motion();
        std::mt19937 rng(g_config.seed);

        auto result = run_approach_guidance(
            x0, n, params, &nav, &rng, 
            g_config.thrust_mag_sigma,
            g_config.thrust_pointing_sigma
        );

        json control_history = json::array();
        json waypoint_history = json::array();

        for (int i = 0; i < result.num_points - 1; ++i) {
            control_history.push_back(vec3_to_json(result.control_history[i]));
            waypoint_history.push_back(vec3_to_json(result.waypoint_history[i]));
        }

        ConeMesh cone = generate_corridor_cone(
            g_config.approach_axis,
            g_config.corridor_angle_rad,
            x0.head<3>().norm()
        );

        json cone_vertices = json::array();

        for (auto& v : cone.vertices) {
            cone_vertices.push_back({v[0], v[1], v[2]});
        }

        json cone_faces = json::array();

        for (auto& f : cone.faces) {
            cone_faces.push_back({f[0], f[1], f[2]});
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
            {"saturationCount", result.saturation_count},
            {"measurementCount", result.measurement_count},
            {"dropoutCount", result.dropout_count},
            {"corridorCone", {
                {"vertices", cone_vertices},
                {"faces", cone_faces}
            }}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(
            json({{"error", e.what()}}).dump(), 
            "application/json"
        );
    }
}

/// @brief POST "/api/monte-carlo"
static void handle_monte_carlo(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        Vec6 x0 = json_to_vec6(body["initialState"]);
        int n_samples = body.value("nSamples", 1000);
        Vec3 pos_error = json_to_vec3(body.at("/uncertainty/posError"_json_pointer));
        Vec3 vel_error = json_to_vec3(body.at("/uncertainty/velError"_json_pointer));

        ApproachParams approach_params;
        UncertaintyModel uncertainty;
        int n_threads;
        unsigned int seed;
        double altitude;

        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            approach_params = g_config.to_approach_params();
            uncertainty = g_config.to_uncertainty();
            n_threads = g_config.n_threads;
            seed = g_config.seed;
            altitude = g_config.altitude;
        }

        uncertainty.pos_error = pos_error;
        uncertainty.vel_error = vel_error;
        double n = OrbitalParams(altitude).mean_motion();

        auto result = run_monte_carlo(
            x0, n, approach_params,
            uncertainty, n_samples, n_threads, seed
        );

        ConeMesh cone = generate_corridor_cone(
            g_config.approach_axis,
            g_config.corridor_angle_rad,
            g_config.success_range
        );
        
        json samples = json::array();
        int limit = std::min(result.n_samples, 500);

        for (int i = 0; i < limit; ++i) {
            samples.push_back({{"success", result.samples[i].success},
                               {"finalRange", result.samples[i].final_range},
                               {"finalVelocity", result.samples[i].final_velocity},
                               {"totalDV", result.samples[i].total_dv},
                               {"duration", result.samples[i].duration},
                               {"saturationCount", result.samples[i].saturation_count},
                               {"failureReason", static_cast<int>(result.samples[i].failure_reason)}
            });
        }

        json final_states = json::array();

        for (int i = 0; i < limit; ++i) {
            final_states.push_back(vec6_to_json(result.samples[i].final_state));
        }

        json cone_vertices = json::array();

        for (auto& v : cone.vertices) {
            cone_vertices.push_back({v[0], v[1], v[2]});
        }

        json cone_faces = json::array();

        for (auto& f : cone.faces) {
            cone_faces.push_back({f[0], f[1], f[2]});
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
            {"dvPercentiles", percentile_stats_to_json(result.dv_percentiles)},
            {"durationPercentiles", percentile_stats_to_json(result.duration_percentiles)},
            {"failures", failure_breakdown_to_json(result.failures)},
            {"samples", samples},
            {"finalStates", final_states},
            {"corridorCone", {
                {"vertices", cone_vertices},
                {"faces", cone_faces}
            }}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief POST "/api/envelope" 
static void handle_envelope_start(const httplib::Request& req, httplib::Response& res) {
    try {
        json body = json::parse(req.body);

        EnvelopeConfig config = parse_envelope_config(body);
        Vec3 pos_error = json_to_vec3(body.at("/uncertainty/posError"_json_pointer));
        Vec3 vel_error = json_to_vec3(body.at("/uncertainty/velError"_json_pointer));

        ApproachParams params;
        UncertaintyModel uncertainty;
        int n_threads;
        unsigned int seed;
        double altitude;

        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            params = g_config.to_approach_params();
            uncertainty = g_config.to_uncertainty();
            n_threads = g_config.n_threads;
            seed = g_config.seed;
            altitude = g_config.altitude;
        }

        uncertainty.pos_error = pos_error;
        uncertainty.vel_error = vel_error;
        config.seed = seed;
        double n = OrbitalParams(altitude).mean_motion();

        // Create job
        auto job = std::make_shared<EnvelopeJob>();
        job->id = next_job_id++;
        job->total_cells = config.range_steps * config.u_max_steps;
        job->result = std::make_shared<PerformanceEnvelope>();

        // Capture what the worker needs
        int job_id = job->id;

        job->worker = std::thread([job, params, uncertainty, config, n, n_threads]() {
            try {
                auto start_time = std::chrono::steady_clock::now();

                PerformanceEnvelope envelope;
                envelope.config = config;
                envelope.params = params;
                envelope.uncertainty = uncertainty;

                // Determine approach axis name
                Vec3 axis = params.corridor_params.approach_axis;
                
                if (std::abs(axis(1)) > 0.9) {
                    envelope.approach_axis = "vbar";
                } else if (std::abs(axis(2)) > 0.9) {
                    envelope.approach_axis = "hbar";
                } else {
                    envelope.approach_axis = "rbar";
                }

                // Build axes
                envelope.range_vals.resize(config.range_steps);
                envelope.u_max_vals.resize(config.u_max_steps);

                for (int i = 0; i < config.range_steps; ++i) {
                    double range_frac = static_cast<double>(i) / std::max(config.range_steps - 1, 1);
                    envelope.range_vals[i] = config.range_min + range_frac * (config.range_max - config.range_min);
                }

                double log_min = std::log10(config.u_max_min);
                double log_max = std::log10(config.u_max_max);

                for (int j = 0; j < config.u_max_steps; ++j) {
                    double u_max_frac = static_cast<double>(j) / std::max(config.u_max_steps - 1, 1);
                    envelope.u_max_vals[j] = std::pow(10.0, log_min + u_max_frac * (log_max - log_min));
                }

                // Allocate grid
                envelope.grid.resize(config.range_steps);
                
                for (int i = 0; i < config.range_steps; ++i) {
                    envelope.grid[i].resize(config.u_max_steps);
                }

                // Thread count
                int threads = n_threads;

                if (threads <= 0) {
                    threads = static_cast<int>(std::thread::hardware_concurrency());

                    if (threads == 0) {
                        threads = 4;
                    }
                }

                int total_cells = config.range_steps * config.u_max_steps;
                std::mutex queue_mutex;
                int next_cell = 0;

                auto worker = [&]() {
                    while (true) {
                        int cell_idx;
                        {
                            std::lock_guard<std::mutex> lock(queue_mutex);

                            if (next_cell >= total_cells) {
                                break;
                            }

                            cell_idx = next_cell++;
                        }

                        int i = cell_idx / config.u_max_steps;
                        int j = cell_idx % config.u_max_steps;

                        unsigned int cell_seed = config.seed + static_cast<unsigned int>(cell_idx * 997);

                        // Build params for cell
                        ApproachParams cell_params = params;
                        cell_params.u_max = envelope.u_max_vals[j];
                        Vec6 x0 = state_from_range(envelope.range_vals[i], cell_params);

                        MonteCarloResult cell_result = run_monte_carlo(
                            x0, n, cell_params, uncertainty,
                            config.samples_per_point, 1, cell_seed
                        );

                        // Condense
                        EnvelopePoint pt;
                        pt.range = envelope.range_vals[i];
                        pt.u_max = envelope.u_max_vals[i];
                        pt.success_rate = cell_result.success_rate;
                        pt.n_success = cell_result.n_success;
                        pt.n_samples = cell_result.n_samples;
                        pt.mean_dv = cell_result.mean_dv;
                        pt.p95_dv = cell_result.dv_percentiles.p95;
                        pt.p99_dv = cell_result.dv_percentiles.p99;
                        pt.mean_duration = cell_result.mean_duration;
                        pt.p95_duration = cell_result.duration_percentiles.p95;
                        pt.failures = cell_result.failures;
                        pt.dominant_failure = dominant_failure_reason(cell_result.failures);

                        {
                            std::lock_guard<std::mutex> lock(queue_mutex);
                            envelope.grid[i][j] = pt;
                        }

                        job->cells_complete++;
                    }
                };

                std::vector<std::thread> threads_vec;

                for (int th = 0; th < threads; ++th) {
                    threads_vec.emplace_back(worker);
                }

                for (auto& t : threads_vec) {
                    t.join();
                }

                auto end_time = std::chrono::steady_clock::now();
                envelope.total_compute_time = std::chrono::duration<double>(end_time - start_time).count();

                *(job->result) = std::move(envelope);
                job->status = "complete";
            } catch (const std::exception& e) {
                job->status = "failed";
                job->error_msg = e.what();
            }
        });

        // Store job
        {
            std::lock_guard<std::mutex> lock(jobs_mutex);
            envelope_jobs[job_id] = job;
        }

        json response = {
            {"jobId", job_id},
            {"totalCells", job->total_cells},
            {"status", "running"}
        };

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief GET "/api/envelope/:id"
static void handle_envelope_status(const httplib::Request& req, httplib::Response& res) {
    try {
        std::string path = req.path;
        int job_id = std::stoi(path.substr(path.rfind('/') + 1));

        std::shared_ptr<EnvelopeJob> job;
        {
            std::lock_guard<std::mutex> lock(jobs_mutex);
            auto it = envelope_jobs.find(job_id);

            if (it == envelope_jobs.end()) {
                res.status = 404;
                res.set_content(json({{"error", "Job not found"}}).dump(), "application/json");
                return;
            }

            job = it->second;
        }

        json response = {
            {"jobId", job->id},
            {"status", job->status},
            {"cellsComplete", job->cells_complete.load()},
            {"totalCells", job->total_cells},
            {"progress", static_cast<double>(job->cells_complete.load()) / std::max(job->total_cells, 1)}
        };

        if (job->status == "failed") {
            response["error"] = job->error_msg;
        }

        if (job->status == "complete") {
            auto& env = *(job->result);

            response["computeTime"] = env.total_compute_time;
            response["approachAxis"] = env.approach_axis;
            response["rangeValues"] = env.range_vals;
            response["uMaxValues"] = env.u_max_vals;

            json grid = json::array();

            for (int i = 0; i < static_cast<int>(env.grid.size()); ++i) {
                for (int j = 0; j < static_cast<int>(env.grid[i].size()); ++j) {
                    auto& pt = env.grid[i][j];
                    grid.push_back({
                        {"rangeIdx", i},
                        {"uMaxIdx", j},
                        {"range", pt.range},
                        {"uMax", pt.u_max},
                        {"successRate", pt.success_rate},
                        {"nSuccess", pt.n_success},
                        {"nSamples", pt.n_samples},
                        {"meanDV", pt.mean_dv},
                        {"p95DV", pt.p95_dv},
                        {"p99DV", pt.p99_dv},
                        {"meanDuration", pt.mean_duration},
                        {"p95Duration", pt.p95_duration},
                        {"failures", failure_breakdown_to_json(pt.failures)},
                        {"dominantFailure", failure_reason_to_string(pt.dominant_failure)}
                    });
                }
            }

            response["grid"] = grid;
        }

        res.set_content(response.dump(), "application/json");
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json({{"error", e.what()}}).dump(), "application/json");
    }
}

/// @brief DELETE "/api/envelope/:id"
static void handle_envelope_delete(const httplib::Request& req, httplib::Response& res) {
    try {
        std::string path = req.path;
        int job_id = std::stoi(path.substr(path.rfind('/') + 1));

        std::shared_ptr<EnvelopeJob> job;
        {
            std::lock_guard<std::mutex> lock(jobs_mutex);
            auto it = envelope_jobs.find(job_id);

            if (it == envelope_jobs.end()) {
                res.status = 404;
                res.set_content(json({{"error", "Job not found"}}).dump(), "application/json");
                return;
            }

            job = it->second;
            envelope_jobs.erase(it);
        }

        json response = {
            {"jobId", job_id},
            {"deleted", true}
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
        double altitude;

        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            altitude = g_config.altitude;
        }

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
    std::string config_path = "gnc_config.yaml";

    if (argc > 1) {
        port = std::stoi(argv[1]);
    }

    if (argc > 2) {
        config_path = argv[2];
    }

    try {
        g_config = load_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return 1;
    }

    // CORS middleware
    svr.set_pre_routing_handler([](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, DELETE");
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

    svr.Get("/api/config", handle_get_config);
    svr.Post("/api/config/reload", handle_reload_config);

    svr.Get("/api/orbit", handle_orbit_info);
    svr.Post("/api/approach-guidance", handle_approach_guidance);
    svr.Post("/api/monte-carlo", handle_monte_carlo);

    svr.Post("/api/envelope", handle_envelope_start);
    svr.Get(R"(/api/envelope/(\d+))", handle_envelope_status);
    svr.Delete(R"(/api/envelope/(\d+))", handle_envelope_delete);

    std::cout << "RelNav-MC API Server v2.0 starting on port " << port << std::endl;
    std::cout << "Endpoints:" << std::endl;
    std::cout << "  GET  /api/health" << std::endl;
    std::cout << "  GET  /api/config" << std::endl;
    std::cout << "  POST /api/config/reload" << std::endl;
    std::cout << "  GET  /api/orbit?altitude=<m>" << std::endl;
    std::cout << "  POST /api/approach-guidance" << std::endl;
    std::cout << "  POST /api/monte-carlo" << std::endl;
    std::cout << "  POST   /api/envelope" << std::endl;
    std::cout << "  GET    /api/envelope/:id" << std::endl;
    std::cout << "  DELETE /api/envelope/:id" << std::endl;

    svr.listen("0.0.0.0", port);

    return 0;
}