#include "sensor_fusion_pkg/kalman_filter.hpp"
KalmanFilter::KalmanFilter(int state_size, int measurement_size)
    : state_size_(state_size),
      measurement_size_(measurement_size)
{
    x_ = Eigen::VectorXd::Zero(state_size_);
    P_ = Eigen::MatrixXd::Identity(state_size_, state_size_) * 1.0;

    I_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
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
    // Tahmin edilen ölçüm
    Eigen::VectorXd z_pred = H * x_;

    // Ölçüm hatası (innovation)
    Eigen::VectorXd y = z - z_pred;

    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    // Use a solver (LDLT) instead of explicit inverse for stability and speed
    Eigen::MatrixXd S_inv = S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    // Kalman Gain
    Eigen::MatrixXd K = P_ * H.transpose() * S_inv;

    // Yeni durum = eski + K * (ölçüm - tahmin)
    x_ = x_ + K * y;

    // Yeni belirsizlik
    P_ = (I_ - K * H) * P_;
}

Eigen::VectorXd KalmanFilter::state() const {
    return x_;
}
