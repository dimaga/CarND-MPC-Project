#include "MPC.h"
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

namespace {
const std::size_t kN = 25;
const double kDt = 0.01;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in
// the simulator around in a circle with a constant steering angle and
// velocity on a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const double kRefCte = 0;
const double kRefEpsi = 0;
const double kRefV = 40;

enum {
  kStartX = 0,
  kStartY = kStartX + kN,
  kStartPsi = kStartY + kN,
  kStartV = kStartPsi + kN,
  kStartCte = kStartV + kN,
  kStartEpsi = kStartCte + kN,
  kStartDelta = kStartEpsi + kN,
  kStartA = kStartDelta + kN - 1
};

AD<double> norm_pi(AD<double> angle_rad) {
  if (angle_rad > M_PI) {
    angle_rad -= 2 * M_PI;
  }

  if (angle_rad < -M_PI) {
    angle_rad += 2 * M_PI;
  }

  return angle_rad;
}

AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

AD<double> polyeval_deriv(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i - 1) * i;
  }
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;

  explicit FG_eval(Eigen::VectorXd coeffs) {
    coeffs_ = coeffs;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg,  // NOLINT(runtime/references)
                  const ADvector& vars) {
    fg[0] = 0;
    for (std::size_t i = 0; i < kN; ++i) {
      fg[0] += CppAD::pow(vars[kStartCte + i] - kRefCte, 2);
      fg[0] += CppAD::pow(vars[kStartEpsi + i] - kRefEpsi, 2);
      fg[0] += CppAD::pow(vars[kStartV + i] - kRefV, 2);
    }

    for (std::size_t i = 0; i < kN - 1; ++i) {
      fg[0] += CppAD::pow(vars[kStartDelta + i], 2);
      fg[0] += CppAD::pow(vars[kStartA + i], 2);
    }

    for (std::size_t i = 0; i < kN - 2; ++i) {
      fg[0] += CppAD::pow(vars[kStartDelta + i + 1] - vars[kStartDelta + i], 2);
      fg[0] += CppAD::pow(vars[kStartA + i + 1] - vars[kStartA + i], 2);
    }

    fg[1 + kStartX] = vars[kStartX];
    fg[1 + kStartY] = vars[kStartY];
    fg[1 + kStartPsi] = vars[kStartPsi];
    fg[1 + kStartV] = vars[kStartV];
    fg[1 + kStartCte] = vars[kStartCte];
    fg[1 + kStartEpsi] = vars[kStartEpsi];

    for (std::size_t i = 0; i < kN - 1; ++i) {
      AD<double> x1 = vars[kStartX + i + 1];
      AD<double> y1 = vars[kStartY + i + 1];
      AD<double> psi1 = vars[kStartPsi + i + 1];
      AD<double> v1 = vars[kStartV + i + 1];
      AD<double> cte1 = vars[kStartCte + i + 1];
      AD<double> epsi1 = vars[kStartEpsi + i + 1];

      AD<double> x0 = vars[kStartX + i];
      AD<double> y0 = vars[kStartY + i];
      AD<double> psi0 = vars[kStartPsi + i];
      AD<double> v0 = vars[kStartV + i];
      AD<double> delta0 = vars[kStartDelta + i];
      AD<double> a0 = vars[kStartA + i];
      AD<double> cte0 = vars[kStartCte + i];
      AD<double> epsi0 = vars[kStartEpsi + i];

      AD<double> f0 = polyeval(coeffs_, x0);
      AD<double> psides0 = CppAD::atan(polyeval_deriv(coeffs_, x0));

      AD<double> psi_offset = v0 / Lf * delta0 * kDt;

      fg[2 + kStartX + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * kDt);
      fg[2 + kStartY + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * kDt);
      fg[2 + kStartPsi + i] = norm_pi(psi1 - (psi0 + psi_offset));
      fg[2 + kStartV + i] = v1 - (v0 + a0 * kDt);
      fg[2 + kStartCte + i] = cte1 - (f0 - y0 + v0 * epsi0 * kDt);
      fg[2 + kStartEpsi + i] = norm_pi(epsi1 - (psi0 - psides0 + psi_offset));
    }
  }
};
}  // namespace


//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const std::size_t n_vars = kN * 6 + (kN - 1) * 2;
  const std::size_t n_constraints = kN * 6;

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  vars[kStartX] = state[0];
  vars[kStartY] = state[1];
  vars[kStartPsi] = state[2];
  vars[kStartV] = state[3];
  vars[kStartCte] = state[4];
  vars[kStartEpsi] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < kStartDelta; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = kStartDelta; i < kStartA; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (int i = kStartA; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[kStartX] = state[0];
  constraints_lowerbound[kStartY] = state[1];
  constraints_lowerbound[kStartPsi] = state[2];
  constraints_lowerbound[kStartV] = state[3];
  constraints_lowerbound[kStartCte] = state[4];
  constraints_lowerbound[kStartEpsi] = state[5];

  constraints_upperbound[kStartX] = state[0];
  constraints_upperbound[kStartY] = state[1];
  constraints_upperbound[kStartPsi] = state[2];
  constraints_upperbound[kStartV] = state[3];
  constraints_upperbound[kStartCte] = state[4];
  constraints_upperbound[kStartEpsi] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  return {solution.x[kStartDelta], solution.x[kStartA]};
}
