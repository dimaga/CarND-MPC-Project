
# CarND-MPC-Project

My solution of CarND-MPC-Project assignment from Udacity Self Driving Car nanodegree course, Term 2. See project assignment starter code in https://github.com/udacity/CarND-MPC-Project

---

## Dependencies

The project compilation and work have been verified under Mac OX Sierra XCode 8.3.1. I used cmake 3.7.2 to build project files.

## Code Style

To enforce [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html), I included Google's `cpplint.py` file available in `./src/Lint` folder. This tool expects installed python2.7. To check style of my code, run the following command line (from `./src/Lint`):

```
./cpplint.py ../*.h ../*.cpp
```

## Rubric Points

### The Model

MPC controller uses simplified Kinematic bicycle model, presented at course lectures and implemented in FG_eval::operator() method of MPC.cpp. The following formulae are applied:

```
psi_offset = velocity / Lf * steering * dt
x1 = x0 + v0 * cos(psi0) * dt
y1 = y0 + v0 * sin(psi0) * dt
psi1 = norm_pi(psi0 + psi_offset)
v1 = v0 + throttle * dt
```

where dt is timestep in seconds, (x, y) are 2D coordinates of the car, psi is the orientation of the car in radians, Lf distance from the car center of mass to front wheels, norm_pi() normalizes the angle to keep it in the range of [-pi, pi].

MPC controller traces the state of the car into the future, trying to minimize the cost function, consisting of sum of weighted squares of:
* cross track errors - distances between car position and the closest point on the lane center
* epsi - differences in radians between the tangent of the closest point of the lane center and the car forward direction
* differences between desired and actual linear velocity to move the car forward
* actuators commands to keep their absolute values small
* differences between consecutive actuator commands to keep them varying smoothly

### Timestep Length and Dt

In my case, default values, presented in MPC lab solution (N = 25, dt = 0.01), showed best results. Other values result in stability problems.

Increasing planning horizon by longer timestep length (kN in MPC.cpp) or smaller update period (kDt) leads to the following issues:
* Longer fitting time, which adds up additional latency to the controller and causes stability problems. I tried to compensate that by measuring performance of my mps Solver() and adding it to the overall latency, see std::chrono usage in MPC::Solve() code of MPC.cpp
* Additional degrees of freedom for the planning problem (higher "variance", "overfitting"). The planning path may introduce circular motion, intersecting road ledges and other parts of the environment - cost function does not fine controller for crossing obstacles
* Update rate of the simulator may not correspond to update rate of the controlling application, which leads to different physics of the car

Decreasing timestep length results in insufficient planning or horizon, not covering latency values.

Decreasing update period (kDt) leads to too coarse planning route, not fully representing the physics of the car

### Polynomial Fitting

Since MPC controller is implemented via ipopt(), minimizing sum of squared errors cost function with respect to some parameters, it is convenient to represent desired trajectory by a differentiable polynomial. I used a cubic polynomial, presented in MPC lab. The fitting procedure is implemented in polyfit() function of main.cpp and sampling methods for polynomial values and its first derivative are implemented in polyeval() and polyeval_deriv() of MPC.cpp module correspondingly.

The polynomial is parametrized by x coordinate, and returns y coordinate. Therefore, referenced trajectory waypoints given in map coordinates are first transformed into car local reference frame, where X-axes points forward in longitudal direction of the car, and Y-axes points in the lateral direction. This also simplifies calculation of cross-track-error (take y-coordinate in the local reference frame of the car) and epsi (take atan() of the polynomial derivative). 

### Model Predictive Control with Latency

Model Predictive Controller builds the track of future states of the car, optimizing cost function by selecting appropriate actuator values along the way. In order to compensate the latency, I return to the simulator actuator commands from the middle of the track, corresponding to the latency timestamp.

The latency compensation is implemented in the end of MPC::Solve() method, in MPC.cpp. It takes into account both constant latency (0.1 seconds) and varying latency caused by MPC solver and measured by std::chrono timers.
