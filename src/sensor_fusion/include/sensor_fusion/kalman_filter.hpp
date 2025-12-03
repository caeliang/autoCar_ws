#pragma once
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(int state_size, int measurement_size);

    void init(const Eigen::VectorXd& x0,
              const Eigen::MatrixXd& P0);

    void predict(const Eigen::MatrixXd& F,
                 const Eigen::MatrixXd& Q);

    void update(const Eigen::VectorXd& z,
                const Eigen::MatrixXd& H,
                const Eigen::MatrixXd& R);

    Eigen::VectorXd state() const;

private:
    int state_size_;
    int measurement_size_;

    Eigen::VectorXd x_;   // Durum vektörü (x, y, yaw, vx, vy...)
    Eigen::MatrixXd P_;   // Durum belirsizlik matrisi

    Eigen::MatrixXd I_;   // Birim matris (işlem kolaylığı için)
    // Temporaries to avoid reallocations during predict/update
    Eigen::VectorXd y_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd HP_; // helper: H * P
};
