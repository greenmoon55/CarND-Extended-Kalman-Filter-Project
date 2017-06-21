#include <math.h>
#include "kalman_filter.h"
#define PI 3.1415926

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
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

double NormalizeAngle(double phi)
{
  const double Max = PI;
  const double Min = -PI;

  return phi < Min
    ? Max + std::fmod(phi - Min, Max - Min)
    : std::fmod(phi - Min, Max - Min) + Min;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double ro = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1), x_(0));
  
  // theta = atan(sin(theta) / cos(theta));
  // while (theta > PI) {
  //   theta -= PI * 2;
  // }
  // while (theta < -PI) {
  //   theta += PI * 2;
  // }
  double ro_dot;
  if (fabs(ro) < 0.0001) {
    ro_dot = 0;
  } else {
    ro_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / ro;
  }
  
  cout << "ro: " << ro << " theta: " << theta << " ro_dot: " << ro_dot << endl;
  VectorXd z_pred = VectorXd(3);
  z_pred << ro, theta, ro_dot;
  VectorXd y = z - z_pred;
  y(1) = NormalizeAngle(y(1));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
