#include "kalman_filter.h"
#include "tools.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
    long x_size = x_.size();
    I = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    long x_size = x_.size();
    I = MatrixXd::Identity(x_size, x_size);

    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();

    x_ += K * y;
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    H_ = Tools::CalculateJacobian(x_);

    long x_size = x_.size();
    I = MatrixXd::Identity(x_size, x_size);

    VectorXd y = z - cartesian2polar(x_);
    while (y(1) > M_PI) y(1) = y(1) - 2 * M_PI;
    while (y(1) < -M_PI) y(1) = y(1) + 2 * M_PI;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();

    x_ += K * y;
    P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::cartesian2polar(const Eigen::VectorXd &x) {
    float px = x(0), py = x(1), vx = x(2), vy = x(3);
    VectorXd h(3);
    float rho = sqrt(px * px + py * py);
    if (rho < 0.001) {
        h << 0, 0, 0;
        return h;
    }
    float phi = atan2(py, px);
    h << rho,
         phi,
            (px * vx + py * vy) / rho;
    return h;
}
