#include "kalman.hpp"
#include <cmath>
#include <algorithm>

KalmanFilter::KalmanFilter() {
    // State: [x, y, vx, vy]
    state_ = Eigen::VectorXd(4);
    state_ << 0, 0, 0, 0;  // Initial state

    // Covariance matrix
    P_ = Eigen::MatrixXd::Identity(4, 4) * 100;

    // State transition matrix 
    F_ = Eigen::MatrixXd::Identity(4, 4);
    

    // Measurement matrix 
    H_ = Eigen::MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    Q_ = Eigen::MatrixXd::Zero(4, 4);
    R_ = Eigen::MatrixXd::Identity(2, 2) * 0.887;
    sigma_a_ = 250.0;
    friction_decel_ = 0.0;
}

void KalmanFilter::reset() {
    state_ = Eigen::VectorXd(4);
    state_ << 0, 0, 0, 0;
    P_ = Eigen::MatrixXd::Identity(4, 4) * 100;

    F_ = Eigen::MatrixXd::Identity(4, 4);
    Q_ = Eigen::MatrixXd::Zero(4, 4);
}

void KalmanFilter::predict(double dt) {
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Q_ << dt4/4.0,     0.0, dt3/2.0,     0.0,
              0.0, dt4/4.0,     0.0, dt3/2.0,
          dt3/2.0,     0.0,     dt2,     0.0,
              0.0, dt3/2.0,     0.0,     dt2;
              
    Q_ *= (sigma_a_ * sigma_a_);
    state_ = F_ * state_;

    if (friction_decel_ > 0.0) {
        double vx = state_(2);
        double vy = state_(3);
        double speed = std::hypot(vx, vy);
        if (speed > 1e-6) {
            double newSpeed = std::max(0.0, speed - friction_decel_ * dt);
            double scale = newSpeed / speed;
            state_(2) = vx * scale;
            state_(3) = vy * scale;
        }
    }

    P_ = F_ * P_ * F_.transpose() + Q_;
}
void KalmanFilter::update(const Eigen::VectorXd& measurement) {
    Eigen::VectorXd y = measurement - H_ * state_;
    
    Eigen::MatrixXd H_P_Ht = H_ * P_ * H_.transpose();
    Eigen::MatrixXd S = H_P_Ht + R_;

    Eigen::MatrixXd K = (P_ * H_.transpose()) * S.ldlt().solve(Eigen::MatrixXd::Identity(2, 2));
    state_ = state_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return state_;
}

void KalmanFilter::setState(const Eigen::VectorXd& state) {
    state_ = state;
}

void KalmanFilter::setF(const Eigen::MatrixXd& F) {
    F_ = F;
}

Eigen::MatrixXd KalmanFilter::getCovariance() const {
    return P_;
}
