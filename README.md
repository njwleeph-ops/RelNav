# RelNav-MC

Proximity operations simulator for spacecraft rendezvous and docking. Closed-loop GNC (EKF + corridor guidance + LQR) with Monte Carlo dispersion analysis and performance envelope mapping.

C++17 / Eigen / React

## build

```bash
mkdir build && cd build
cmake ..
make
./relnav-server
```

Frontend runs on `localhost:3000`, backend on `localhost:8080`.