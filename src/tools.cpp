#include "tools.h"
#include <iostream>


Eigen::VectorXd 
Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                     const std::vector<Eigen::VectorXd> &ground_truth) {
  /**
   * Evaluating KF Performance Part 1
   * 24-23
   * Calculate the RMSE.
   */
  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data\n";
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}


Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state) {
  /**
   * lesson 24-19: Jacobian Matrix Part 1
   */
  Eigen::MatrixXd Hj(3,4);
  // recover state parameters
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);
  
  double px2_py2 = px*px+py*py;
  
  // check division by zero
  if( px2_py2 < 1e-3 ) {
     //std::cout<<"ERROR: Probably Devision by zero\n";
     px2_py2 = 1.0/3.0;
  }

  // compute the Jacobian matrix  
  Hj << px/sqrt(px2_py2),  py/sqrt(px2_py2), 0, 0,
       -py/(px2_py2),     px/(px2_py2),     0, 0,
       py*(vx*py-vy*px)/sqrt((px2_py2)*(px2_py2)*(px2_py2)),
       px*(vy*px-vx*py)/sqrt((px2_py2)*(px2_py2)*(px2_py2)),
       px/sqrt(px2_py2),  py/sqrt(px2_py2);
  return Hj;
}
