#include "sensor_fusion/kalman_filter.hpp"
KalmanFilter::KalmanFilter(int state_size, int measurement_size)
    : state_size_(state_size),
      measurement_size_(measurement_size)
{
    x_ = Eigen::VectorXd::Zero(state_size_);
    P_ = Eigen::MatrixXd::Identity(state_size_, state_size_) * 1.0;

    I_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
    // Allocate temporaries based on sizes
    y_ = Eigen::VectorXd::Zero(measurement_size_);
    S_ = Eigen::MatrixXd::Zero(measurement_size_, measurement_size_);
    K_ = Eigen::MatrixXd::Zero(state_size_, measurement_size_);
    HP_ = Eigen::MatrixXd::Zero(measurement_size_, state_size_);
}

void KalmanFilter::init(const Eigen::VectorXd& x0,
                        const Eigen::MatrixXd& P0)
{
    x_ = x0;
    P_ = P0;
}

void KalmanFilter::predict(const Eigen::MatrixXd& F,
                           const Eigen::MatrixXd& Q)
{
    // Yeni durum = F * eski durum
    x_ = F * x_;

    // Yeni belirsizlik = F * P * Fᵀ + Q
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R)
{
    // Predicted measurement and innovation (reuse temporaries)
    // z_pred = H * x_
    y_.noalias() = z - (H * x_);

    // Innovation covariance S = H * P * H^T + R
    // Compute H * P once into HP_
    HP_.noalias() = H * P_;
    S_.noalias() = HP_ * H.transpose();
    S_ += R;

    // Compute Kalman gain K = P * H^T * S^{-1}
    // Instead of forming S^{-1}, solve S * X = (H * P)^T for X then transpose.
    // temp = S.ldlt().solve(HP_)
    Eigen::MatrixXd temp = S_.ldlt().solve(HP_);
    K_.noalias() = temp.transpose();

    // Update state: x = x + K * y
    x_.noalias() += K_ * y_;

    // Update covariance: P = (I - K * H) * P
    Eigen::MatrixXd I_KH = I_ - K_ * H;
    P_.noalias() = I_KH * P_;
}

Eigen::VectorXd KalmanFilter::state() const {
    return x_;
}
