#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //set state dimension
  int n_x_ = 5;

  //set augmented dimension
  int n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_ = 3;

  //define spreading parameter
  double lambda_ = 3 - n_aug_;

  //define vector for weights
  VectorXd weights_ = VectorXd(2 * n_aug_ + 1);

  //define matrix for predicted sigma points
  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // initialize variables needed
    // v , yaw , yawd are needed to tune
    x_ << 1, 1, 0, 0, 0;

    // for LIDAR data
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);

      x_ << px, py, 0, 0, 0;
    }
    // for RADAR data
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double ro = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double ro_dot = meas_package.raw_measurements_(2); // not used

      double px = ro*cos(phi);
      double py = ro*sin(phi);

      x_ << px, py, 0, 0, 0;
    }

    P_ << 1., 0., 0., 0., 0.,
          0., 1., 0., 0., 0.,
          0., 0., 1., 0., 0.,
          0., 0., 0., 1., 0.,
          0., 0., 0., 0., 1.;

    // initialize previous time
    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
    return;
  }

  // prediction
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // delta_t - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  // update
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  ///////////////////////////
  // generate sigma points
  ///////////////////////////

  // create augmented state vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);

  // matrix Q
  MatrixXd Q(2,2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;

  P_aug.topLeftCorner(5,5) = P;
  P_aug.bottomRightCorner(2,2) = Q;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;

  // create augmented sigma points
  for(int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(n_aug_+i+1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /////////////////////////////
  // calculate predicted state vector from augmented sigma points
  /////////////////////////////

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // extract values for better readability
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_pred, py_pred;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_pred = px + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_pred = py + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_pred = px + v*delta_t*cos(yaw);
      py_pred = py + v*delta_t*sin(yaw);
    }

    double v_pred = v;
    double yaw_pred = yaw + yawd*delta_t;
    double yawd_pred = yawd;

    // add noise
    px_pred += 0.5*nu_a*(delta_t*delta_t)*cos(yaw);
    py_pred += 0.5*nu_a*(delta_t*delta_t)*sin(yaw);
    v_pred += nu_a*delta_t;
    yaw_pred += 0.5*nu_yawdd*(delta_t*delta_t);
    yawd_pred += nu_yawdd*delta_t;

    // write calculated values to the matrix for pretdicted sigma points
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }

  //////////////////////////////
  // compute predicted mean and covariance
  //////////////////////////////

  // set weights_
  weights_.fill(0.5/(n_aug_+lambda_));
  weights_(0) = lambda_/(lambda_+n_aug_);

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // normalize angle of yaw
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  VectorXd z = meas_package.raw_measurements_;

  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
