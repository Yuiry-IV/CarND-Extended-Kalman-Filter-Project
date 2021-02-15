#include "kalman_filter.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;

  P_ = ( F_ * P_ * F_.transpose() ) + Q_;
}

void KalmanFilter::Update(const VectorXd &z, 
               const Eigen::MatrixXd &H_,  
               const Eigen::MatrixXd &R_ ) {
  /**
   * Update the state by using Kalman Filter equations
   */
  VectorXd y      = z - (H_ * x_);
  
  UpdateCommon(y, H_, R_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, 
               const Eigen::MatrixXd &H_,  
               const Eigen::MatrixXd &R_) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
   
  const double px = x_(0);
  const double py = x_(1);
  const double vx = x_(2);
  const double vy = x_(3);
  
  const double rho     = sqrt(px*px + py*py);
  const double theta   = atan2(py, px);
  
  /**
   * :seedling: Whenever there is division, it’s always a good idea to protect against division by zero. 
   * There are several ways to perform this protection here is one:
   * rho_dot = (px*vx + py*vy) / std::max(eps, rho); with some small eps. 
   */
  const eps = 1e-6;
  const double rho_dot =  (px*vx + py*vy) / std::max(rho, eps);
 
  VectorXd h = VectorXd(3);

  h << rho, theta, rho_dot;

  VectorXd y = z - h;
  
  /**
   * Normalize theta to [-2pi, 2pi]
   * :seedling: There is a mathematical trick that you can use here: For an angle phi: Get the tan(-1)(tan(phi))
   */
   y(1) = atan2(sin(y(1)), cos(y(1)));
   
  UpdateCommon(y,H_,R_);
}

void KalmanFilter::UpdateCommon(const VectorXd &y, 
               const Eigen::MatrixXd &H_,  
               const Eigen::MatrixXd &R_) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S   = ( H_ * P_ * Ht ) + R_;  
  MatrixXd K   = ( P_ * Ht ) * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
    
  const unsigned long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = ( I - (K * H_) ) * P_;
}

