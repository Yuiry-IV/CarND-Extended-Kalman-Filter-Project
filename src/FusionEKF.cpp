#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
              
  // Initial covariance Matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  /**
   * Initializing the FusionEKF.
   * Set the process and measurement noises
   */
  MatrixXd P = MatrixXd(4,4);
  P       << 1,   0,   0,      0,
             0,   1,   0,      0,
             0,   0,   1000,   0,
             0,   0,   0,   1000;
             
  ekf_.set_P(P);
             
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    // Initialize state.
    VectorXd vector_x = VectorXd(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho     = measurement_pack.raw_measurements_[0]; // range
      double phi     = measurement_pack.raw_measurements_[1]; // bearing
      double rho_dot = measurement_pack.raw_measurements_[2]; // rate

      cout << "rho: " << rho << endl;
      cout << "phi: " << phi << endl;
      cout << "rho_dot: " << rho_dot << endl;

      /**
       *      Suggestion
       * Here there is no need for normalization. Mainly normalization is needed after each calculation.
       * On the other hand, you are doing a protective algorithm to ensure all inputs are within the valid range. 
       * Usually we expect the inputs to be correct.
       * Normalize phi to [-pi, pi]
       */
      while (phi > 2.0*M_PI)  phi -= 2.0 * M_PI;
      while (phi < -2.0*M_PI) phi += 2.0 * M_PI;

      /**
       * It is a great idea to use rho_dot for computing rough initial estimates for vx and vy! 
       * Convert each coordinate
       */
      double x  = rho * cos(phi);
      double y  = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);

      vector_x << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // No conversions required for laser measurement
      double x = measurement_pack.raw_measurements_[0];
      double y = measurement_pack.raw_measurements_[1];
      vector_x << x, y, 0, 0;
    }

    ekf_.set_x(vector_x);
    
    // Initial results
    cout << "EKF init: " << ekf_.get_x() << endl;
    // Save initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix Q.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_);
  dt /= 1000000.0; // dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transiction Matrix update
  MatrixXd F = MatrixXd(4, 4);
  F << 1,  0,  dt, 0,
       0,  1,  0,  dt,
       0,  0,  1,  0,
       0,  0,  0,  1;
       
  ekf_.set_F(F);

  // Compute the Noise Covatiance Matrix Q
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  // Pre-calculate some variables for the matrix
  double dt_2   = dt * dt;
  double dt_3   = dt_2 * dt;
  double dt_4   = dt_3 * dt;

  MatrixXd Q = MatrixXd(4, 4);
  Q << dt_4 / 4.0 * noise_ax, 0, dt_3 / 2.0 * noise_ax, 0, 
       0, dt_4 / 4 * noise_ay, 0, dt_3 / 2.0 * noise_ay, 
       dt_3 / 2.0 * noise_ax, 0, dt_2* noise_ax, 0, 
       0, dt_3 / 2.0 * noise_ay, 0, dt_2 * noise_ay;
  ekf_.set_Q(Q);
       
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices P.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Eigen::MatrixXd Hj = Tools::CalculateJacobian( ekf_.get_x() );
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj, R_radar_ );
  } else if ( measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_ );
  } else {
    cout << " somthing else:"  << endl;
    exit(1);
  }

   /*
  // print the output
  cout << ( measurement_pack.timestamp_ ) 
       << " x_ = " << ekf_.x_(0)
       <<","<< ekf_.x_(1) 
       <<","<< ekf_.x_(2) 
       <<","<< ekf_.x_(3) 
       << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
  */
}
