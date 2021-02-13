#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param H measurement matrix
   * @param R measurement covariance matrix
   */
  void Update( const Eigen::VectorXd &z, 
               const Eigen::MatrixXd &H,  
               const Eigen::MatrixXd &R );

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param H measurement matrix
   * @param R measurement covariance matrix
   */
  void UpdateEKF(const Eigen::VectorXd &z,
               const Eigen::MatrixXd &H,  
               const Eigen::MatrixXd &R );

  /**
   * return state vector intem by index
   * @param index - state item index 
   */
  const double get_xi(unsigned index) const {
      return x_(index);
  }
   
  /**
   * return state vector
   */
  const Eigen::VectorXd get_x( ) const {
      return x_;
  } 


  /**
   * set state vector
   */
  void set_x(const Eigen::MatrixXd &x){
      x_ = x;
  }

  /**
   * set state transition matrix
   */  
  void set_F(const Eigen::MatrixXd &F){
      F_ = F;
  }
    
  /**
   * set process covariance matrix
   */
  void set_Q(const Eigen::MatrixXd &Q){
      Q_ = Q;
  }
  
  /**
   * set state covariance matrix
   */ 
  void set_P(const Eigen::MatrixXd &P){
      P_ = P;
  }

private:
  /**
   * Updates of a common part of the the state using  by both Kalman Filter 
   * equations:
   * @param y the intermediate measurement at k+1
   * @param H measurement matrix
   * @param R measurement covariance matrix
   */
  void UpdateCommon(const Eigen::VectorXd &y, 
                    const Eigen::MatrixXd &H_,  
                    const Eigen::MatrixXd &R_ );

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

};

#endif // KALMAN_FILTER_H_
