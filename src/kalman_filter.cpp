#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// void kalmanfilter::init(vectorxd &x_in, matrixxd &p_in, matrixxd &f_in,
                        // matrixxd &h_in, matrixxd &r_in, matrixxd &q_in) {
  // x_ = x_in;
  // p_ = p_in;
  // f_ = f_in;
  // h_ = h_in;
  // r_ = r_in;
  // q_ = q_in;
// }

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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  #define PI 3.14159265;
  
  VectorXd z_pred;
  z_pred = VectorXd(3);
  z_pred(0) = sqrt (x_(0)*x_(0) + x_(1)*x_(1));
  z_pred(1) = atan2 (x_(1), x_(0));
  z_pred(2) = (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt (x_(0)*x_(0) + x_(1)*x_(1));
  VectorXd y = z - z_pred;
  
  //Adjust phi in y to stay inside -PI and PI
  while ((y(1) > 3.14159265) || (y(1) < -3.14159265))
  {
    if(y(1) > 3.14159265)
    {
      y(1) = y(1) - 2 * PI;
    }
    else
    {
      y(1) = y(1) + 2 * PI;
    }
  }

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
