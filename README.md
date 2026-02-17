# RelNav-MC

Spacecraft proximity operations simulator with closed-loop approach guidance and Monte Carlo dispersion analysis. Built as a portfolio project to demonstrate GNC fundamentals for rendezvous and docking scenarios.

## What This Is

RelNav-MC simulates relative motion between two spacecraft in close proximity using the Clohessy-Wiltshire (CW) equations. These linearized equations describe how a "chaser" spacecraft moves relative to a "target" in a local reference frame (LVLH: Local Vertical Local Horizontal).

The simulator provides three analysis tools:

**Approach Guidance** — The core simulation. A closed-loop LQR controller drives the chaser toward the target while respecting an approach corridor cone and glideslope velocity constraint. Each timestep: check corridor position, compute a waypoint (corridor edge if outside, origin if inside), solve LQR toward that waypoint, clamp for glideslope, saturate thrust, RK4 step. Reports success/failure, total ΔV, duration, and control saturation count.

**Propagator Validation** — Analytical (STM) vs numerical (RK4) cross-reference. Propagate any initial condition with the CW state transition matrix, then independently with a fourth-order Runge-Kutta integrator, and compare. Confirms agreement to ~1e-9 m. This is the foundation everything else builds on.

**Monte Carlo** — Run the full approach guidance loop thousands of times with dispersed initial states and thrust errors. Answers the operational question: "Given realistic navigation and thruster uncertainties, what fraction of approaches succeed?" Reports success rate, ΔV statistics, duration distribution, and final position scatter.

## What Changed (v2.0)

The v1.0 simulator had four separate tools (propagator, Monte Carlo on free drift, two-impulse targeting, LQR with glideslope checking). They were demonstrations — hardcoded scenarios showing that each algorithm worked in isolation.

v2.0 collapses these into a single closed-loop guidance system:

- **Approach corridor** is now a 3D cone with a configurable axis (V-bar, R-bar, or H-bar approach), half-angle, and glideslope gain. The controller computes edge waypoints when outside the corridor and drives directly to target when inside.
- **Glideslope constraint** is enforced at every timestep, not just checked after the fact. The control vector's approach component is clamped so approach velocity never exceeds k × range.
- **Monte Carlo** now disperses the full closed-loop guidance run, not just a two-impulse transfer. Each sample gets perturbed initial conditions and thrust errors applied at every control step. This is operationally meaningful — it answers go/no-go questions.
- **Two-impulse targeting** and the old free-drift Monte Carlo are removed from the API. The CW Lambert solver still exists in the codebase for the test suite.
- **Trajectory storage** uses fixed-size arrays (`MAX_TRAJECTORY_POINTS`) to avoid heap allocation in the inner loop.

## The Physics

The CW equations assume:
- Target is in a circular orbit
- Chaser is close enough that gravity gradient is linear
- No perturbations (J2, drag, third body)

In the LVLH frame, x points radial (R-bar), y points along-track (V-bar), z points cross-track (H-bar). The equations couple radial and along-track motion through Coriolis terms, producing the characteristic elliptical drift patterns.

State transition matrix Φ(t) propagates [x, y, z, vx, vy, vz] forward analytically. The numerical integrator (RK4) validates against this to ~1e-9 m error.

## Running It

**Backend (C++):**
```bash
cd backend
mkdir build && cd build
cmake ..
make
./relnav-server
```

**Frontend (React):**
```bash
cd frontend
npm install
npm start
```

Open `http://localhost:3000`. Backend runs on port 8080.

### Dependencies

Backend: Eigen3, nlohmann/json, cpp-httplib (header-only, included)

Frontend: React, react-plotly.js

## Input Reference

### Approach Guidance

| Input | Description |
|-------|-------------|
| x, y, z | Initial position offset from target [m]. x=radial, y=along-track, z=cross-track |
| vx, vy, vz | Initial velocity [m/s] |
| Approach Axis | Corridor direction: V-bar (along-track), R-bar (radial), or H-bar (cross-track) |
| Corridor Angle | Corridor cone half-angle [rad]. 0.175 ≈ 10° |
| Glideslope k | Velocity limit as fraction of range. v_approach ≤ k × range |
| Min Range | Range below which glideslope is relaxed [m]. Avoids singularity at origin |
| Q_pos | LQR state cost on position error. Higher = more aggressive correction |
| Q_vel | LQR state cost on velocity. Higher = penalizes fast approach |
| R | LQR control cost. Higher = less fuel, slower convergence |
| u_max | Thrust saturation limit [m/s²] |
| dt | Integration timestep [s] |
| Timeout | Maximum simulation duration [s] |
| Success Range | Arrival criterion — range to target [m] |
| Success Velocity | Arrival criterion — speed [m/s] |

Tuning: Start with Q_pos=10, Q_vel=50, R=1. Increase R for fuel-optimal. Decrease dt for accuracy (at cost of compute time for Monte Carlo).

### Propagator Validation

| Input | Description |
|-------|-------------|
| x, y, z, vx, vy, vz | Initial state [m, m/s] |
| Duration | Propagation time in orbital periods |
| Preset | Common initial conditions: Football (R-bar offset), Teardrop (V-bar offset), V-bar Hop, Cross-track |
| Validation Steps | Number of RK4 steps for error comparison |

### Monte Carlo

All approach guidance inputs, plus:

| Input | Description |
|-------|-------------|
| Position Error | 1σ navigation uncertainty in position [m] |
| Velocity Error | 1σ navigation uncertainty in velocity [m/s] |
| Thrust Mag Error | 1σ fractional error in thrust magnitude (0.02 = 2%) |
| Thrust Pointing Error | 1σ pointing error [rad] |
| Samples | Number of Monte Carlo runs. 500+ for rough statistics, 2000+ for convergence |
| Seed | RNG seed for reproducibility |

## API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/health` | Health check. Returns `{ status, version }` |
| GET | `/api/orbit?altitude=<m>` | Orbital parameters for given altitude |
| POST | `/api/propagate` | CW analytical propagation |
| POST | `/api/validate` | Analytical vs numerical cross-reference |
| POST | `/api/approach-guidance` | Closed-loop approach simulation |
| POST | `/api/monte-carlo` | Monte Carlo dispersion analysis |

## Project Structure

```
├── backend/
│   ├── include/
│   │   ├── cw_dynamics.hpp            # CW equations, STM, RK4, Trajectory struct
│   │   ├── gnc_algorithms.hpp         # Corridor, glideslope, LQR, approach guidance
│   │   ├── monte_carlo.hpp            # Uncertainty model, sampling, MC analysis
│   │   ├── cpp-httplib/               # httplib.h header
│   │   └── json/                      # nlohmann json header
│   ├── src/
│   │   ├── cw_dynamics.cpp
│   │   ├── gnc_algorithms.cpp
│   │   ├── monte_carlo.cpp
│   │   ├── main.cpp                   # REST API server
│   │   └── tests/                     # Validation suite
│   │       ├── guidance_tests.cpp     # GNC Algo unit tests
│   │       └── monte_carlo_tests.cpp  # Monte Carlo analysis unit tests
│   └── CMakeLists.txt
├── frontend/
│   ├── src/
│   │   ├── api.js                     # Backend API calls
│   │   ├── components/
│   │   │   ├── Header.jsx
│   │   │   ├── TabNav.jsx
│   │   │   ├── GuidancePanel.jsx
│   │   │   ├── ValidationPanel.jsx
│   │   │   └── MonteCarloPanel.jsx
│   │   ├── constants.js               # Plotly theme, ISS defaults
│   │   ├── App.jsx
│   │   └── App.css
│   ├── public/
│   │   └── index.html
│   └── package.json
├── README.md
└── .gitignore
```

## Validation

The test suite (`test_dynamics`) verifies:
- Analytical vs numerical propagator agreement (<1e-9 m)
- State transition matrix properties (Φ(0) = I, det(Φ) = 1)
- Corridor angle computation and edge waypoint geometry
- Glideslope constraint enforcement
- LQR gain convergence (CARE solver)
- Approach guidance termination (success within radius and velocity thresholds)

## References

**Clohessy-Wiltshire Equations:**
Clohessy, W.H. and Wiltshire, R.S., "Terminal Guidance System for Satellite Rendezvous," Journal of the Aerospace Sciences, Vol. 27, No. 9, 1960, pp. 653-658.

**Textbook Treatment:**
Vallado, D.A., "Fundamentals of Astrodynamics and Applications," 4th ed., Microcosm Press, 2013. Chapter 6 covers relative motion.

**LQR for Spacecraft:**
Wie, B., "Space Vehicle Dynamics and Control," 2nd ed., AIAA, 2008.

**NASA Proximity Operations:**
"Crew Transportation Technical Standards and Design Evaluation Criteria," CCT-STD-1140. Defines approach corridors and glideslope constraints for ISS visiting vehicles.

## License

MIT
