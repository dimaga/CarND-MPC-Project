
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

MPC controller traces the state of the car into the future, trying to minimize the cost function, consisting of sum of:
* squared cross track error - distance between car position and the closes point on the lane center
* squared epsi - difference in direction of the tangent of the closest point of the lane center and the car
* squared difference between desired and actual linear velocity to move the car forward
* squared actuator commands to keep them small
* squared difference between consecutive actuator commands to keep them constant

### Timestep Length and Frequency



### Polynomial Fitting

### Model Predictive Control with Latency
