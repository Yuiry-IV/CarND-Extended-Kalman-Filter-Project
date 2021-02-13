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
  const double rho_dot =  (px*vx + py*vy) / rho;
 
  VectorXd h = VectorXd(3);

  h << rho, theta, rho_dot;

  VectorXd y = z - h;
  
  // Normalize theta to [-2pi, 2pi]
  while (y(1) >  2.0*M_PI) y(1) -= 2.0 * M_PI;
  while (y(1) < -2.0*M_PI) y(1) += 2.0 * M_PI;
  
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

