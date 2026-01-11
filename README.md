# RelNav-MC

Spacecraft proximity operations simulator with Monte Carlo dispersion analysis. Built as a portfolio project to demonstrate GNC fundamentals for rendezvous and docking scenarios.

## What This Is

RelNav-MC simulates relative motion between two spacecraft in close proximity using the Clohessy-Wiltshire (CW) equations. These linearized equations describe how a "chaser" spacecraft moves relative to a "target" in a local reference frame (LVLH: Local Vertical Local Horizontal).

The simulator provides four analysis tools:

**CW Propagator** — Free-drift relative motion. Drop a chaser at some offset from the target and watch how orbital mechanics causes it to drift. The classic "football" orbit appears when you start with a pure radial offset.

**Monte Carlo Analysis** — Plan a two-impulse transfer, then ask: "Given my navigation uncertainty and thruster errors, what's my probability of landing inside an approach corridor?" Runs thousands of dispersed samples to answer this statistically.

**Two-Impulse Targeting** — Solve the CW Lambert problem: compute the ΔV pair needed to get from point A to point B in a given time. Includes a sweep plot showing how ΔV cost varies with transfer time.

**LQR Control** — Continuous optimal control approach using Linear Quadratic Regulator. Drives the chaser toward the target while minimizing a cost function. Includes glideslope constraint checking (approach velocity must stay below k × range).

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
./server 
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

### CW Propagator

| Input | Description |
|-------|-------------|
| x, y, z | Initial position offset from target [m]. x=radial, y=along-track, z=cross-track |
| vx, vy, vz | Initial velocity [m/s] |
| Duration | Propagation time in orbital periods |
| Preset | Common initial conditions (Football, Teardrop, V-bar Hop, etc.) |

### Monte Carlo

| Input | Description |
|-------|-------------|
| Initial State | Where the chaser starts |
| Target Position | Where you want to arrive (usually origin) |
| TOF | Time of flight for the transfer [orbits] |
| Samples | Number of Monte Carlo runs (1000+ for good statistics) |
| Corridor | Radius of cylindrical approach corridor [m] |
| Position Error | 1σ navigation uncertainty in position [m] |
| Velocity Error | 1σ navigation uncertainty in velocity [m/s] |
| Thrust Mag Error | 1σ fractional error in thrust magnitude (0.03 = 3%) |
| Thrust Point Error | 1σ pointing error [rad] |

### Two-Impulse

| Input | Description |
|-------|-------------|
| Initial State | Starting position and velocity |
| Target Position | Desired arrival point |
| TOF | Transfer time — shorter = higher ΔV, longer = lower ΔV (usually) |

### LQR Control

| Input | Description |
|-------|-------------|
| Initial State | Starting position |
| Q_pos | State cost weight on position error. Higher = more aggressive correction |
| Q_vel | State cost weight on velocity. Higher = smoother approach |
| R | Control cost weight. Higher = less fuel usage, slower convergence |
| u_max | Control saturation limit [m/s²] |
| Glideslope k | Velocity limit as fraction of range. v_approach ≤ k × range |

Tuning LQR: Start with Q_pos=100, Q_vel=10, R=1000. Increase R for fuel-optimal. Increase Q_pos for faster convergence.

## Project Structure
```
├── backend/
│   ├── include/
│   │   ├── cw_dynamics.hpp      # CW equations, STM, propagators
│   │   ├── monte_carlo.hpp      # Uncertainty sampling, ensemble propagation
│   │   ├── gnc_algorithms.hpp   # Two-impulse, LQR, glideslope
│   │   ├── cpp-httplib          # httplib.h header inside
│   │   └── json                 # nlohmann json header inside json/include/
│   ├── src/
│   │   ├── cw_dynamics.cpp
│   │   ├── monte_carlo.cpp
│   │   ├── gnc_algorithms.cpp
│   │   ├── main.cpp           # REST API
│   │   └── test_dynamics.cpp    # Validation suite
│   └── CMakeLists.txt
├── frontend/
│   ├── src/
│   │   ├── api.js               # Backend API calls
│   │   ├── App.jsx
│   │   └── Components/
│   │       ├── PropagatorPanel.jsx
│   │       ├── MonteCarloPanel.jsx
│   │       ├── TwoImpulsePanel.jsx
│   │       └── LQRPanel.jsx
│   ├── public/
│   │   └── index.html
│   ├── node_modules/       # Created by npm install (default)
│   ├── package-lock.json   # Created by npm install (default) 
│   └── package.json
├── README.md
├── CMakeLists.txt
└── .gitignore
```

## Validation

The test suite (`test_dynamics`) verifies:
- Analytical vs numerical propagator agreement (<1e-9 m)
- Monte Carlo convergence to analytical covariance (<2% error at N=2000)
- Two-impulse arrival accuracy (machine precision)
- State transition matrix properties (Φ(0)=I, det(Φ)=1)

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