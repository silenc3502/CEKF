#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define EPSILON	0.0001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  /* It comes from lecture 11.
     x, y and each displacement, velocity and there are no Z.
   */

#if 1
  H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
#endif

  /* It comes from lecture 18.
     partial derivative of velocity is 0.
  Hj_ << 1, 1, 0, 0,
        1, 1, 0, 0,
        1, 1, 1, 1; */

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // range
      float rho = measurement_pack.raw_measurements_[0];
      // bearing
      float phi = measurement_pack.raw_measurements_[1];
      // range rate
      float rho_dot = measurement_pack.raw_measurements_[2];

      // polar 2 cartesian
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      ekf_.x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */

      ekf_.x_ << measurement_pack.raw_measurements_[0],
		measurement_pack.raw_measurements_[1],
		0,
		0;
    }

#if 0
    if(fabs(ekf_.x_(0)) < EPSILON && fabs(ekf_.x_(1)) < EPSILON)
    {
      ekf_.x_(0) = EPSILON;
      ekf_.x_(1) = EPSILON;
    }
#endif

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

    cout << ekf_.x_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // Discrete Time - Sampling Time at Digital Signal Processing
  // Computer can't calculate calculus.
  // (It's not continuous)
  // So we need to use these DSP numerical method.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_);
  // We want to use dt to us(micro seconds).
  // And first case of dt will 0.
  dt /= pow(10.0, 6);
  previous_timestamp_ = measurement_pack.timestamp_;

  /* It comes from lecture 8 - F Matrix - and lecture 9 - Q Matrix.
   */
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  float dt2 = pow(dt, 2);
  float dt3_div2 = pow(dt, 3) / 2;
  float dt4_div4 = pow(dt, 4) / 4;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4_div4 * noise_ax,	0,	dt3_div2 * noise_ax,	0,
		0,	dt4_div4 * noise_ay,	0,	dt3_div2 * noise_ay,
		dt3_div2 * noise_ax,	0,	dt2 * noise_ax,		0,
		0, 	dt3_div2 * noise_ay, 	0, 	dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
