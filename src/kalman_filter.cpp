#include "kalman_filter.h"

#define EPSILON	0.0001
#define PI2	2 * M_PI;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  /* It comes from lecture 6 KF Prediction step */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  /* It comes from lecture 6 KF Measurement update step */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  /* It comes from lecture 20 EKF Equations */
  float px, py, vx, vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];

  float rho, phi, rho_dot;
  rho = sqrt(pow(px, 2) + pow(py, 2));

  if(rho < 0.000001)
    rho = 0.000001;

  phi = atan2(py, px);
  rho_dot = (px * vx + py * vy) / rho;

#if 0
  double rho = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
  double theta = atan(x_(1) / x_(0));

  if(rho < pow(EPSILON, 2))
    rho = pow(EPSILON, 2);

  double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
#endif

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  while(y(1) > M_PI)
    y(1) -= PI2;

  while(y(1) < -M_PI)
    y(1) += PI2;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
