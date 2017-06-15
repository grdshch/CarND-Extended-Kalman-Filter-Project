#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**
 * A helper method to calculate RMSE.
 */
VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

/**
 * A helper method to calculate Jacobians.
 */
MatrixXd CalculateJacobian(const VectorXd& x_state);

/** Convert vector from cartesian coordinates to polar coordinates
 * @param x - vector (px, py, vx, vy) in cartesian coordinates
 * @return vector (rho, phi, rate) in polar coordiantes
 */
Eigen::VectorXd Cartesian2Polar(const Eigen::VectorXd &x);

#endif /* TOOLS_H_ */
