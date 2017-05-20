#ifndef SRC_MPC_H_
#define SRC_MPC_H_

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  std::vector<double> x_vals_;
  std::vector<double> y_vals_;
};

#endif  // SRC_MPC_H_
