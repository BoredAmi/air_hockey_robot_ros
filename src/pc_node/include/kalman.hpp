#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <eigen3/Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void predict(double dt);
    void update(const Eigen::VectorXd& measurement);
    Eigen::VectorXd getState() const;
    void setState(const Eigen::VectorXd& state);
    void setF(const Eigen::MatrixXd& F);
    void reset();
    Eigen::MatrixXd getCovariance() const;
    double getSigmaA() const { return sigma_a_; }

private:
    double sigma_a_;
    double friction_decel_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;  // Covariance
    Eigen::MatrixXd F_;  // State transition
    Eigen::MatrixXd H_;  // Measurement matrix
    Eigen::MatrixXd Q_;  // Process noise
    Eigen::MatrixXd R_;  // Measurement noise
};

#endif // KALMAN_HPP