#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.empty() || estimations.size() != ground_truth.size()) {
      std::cout << "Error: estimations and ground_truth must have equal sizes" << std::endl;
      return rmse;
  }

  for (size_t i = 0; i < estimations.size(); ++i) {
      VectorXd error = estimations[i] - ground_truth[i];
      error = error.array() * error.array();
      rmse += error;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  double px = x_state(0), py = x_state(1), vx = x_state(2), vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
        -(py / c1), (px / c1), 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
  return Hj;
}

Eigen::VectorXd Cartesian2Polar(const Eigen::VectorXd &x) {
  double px = x(0), py = x(1), vx = x(2), vy = x(3);

  VectorXd h(3);
  float rho = sqrt(px * px + py * py);

  // avoid division by zero
  if (rho < 0.001) {
    h << 0, 0, 0;
    return h;
  }

  float phi = atan2(py, px);
  h << rho, phi, (px * vx + py * vy) / rho;
  return h;
}
