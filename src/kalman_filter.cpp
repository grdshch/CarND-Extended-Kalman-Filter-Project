#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;

  CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd y = z - Cartesian2Polar(x_);

  // put angle into [-PI, PI] interval
  while (y(1) > M_PI) y(1) = y(1) - 2 * M_PI;
  while (y(1) < -M_PI) y(1) = y(1) + 2 * M_PI;

  CommonUpdate(y);
}

void KalmanFilter::CommonUpdate(const Eigen::VectorXd &y)
{
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ += K * y;
  P_ = (I_ - K * H_) * P_;
}


