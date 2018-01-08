#include "ukf.h"
#include "tools.h"
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
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.1;
  
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

  previous_timestamp_ = 0.0;

  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // define vector for weights
  weights_ = VectorXd::Zero(2 * n_aug_ + 1);

  // define matrix for predicted sigma points
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
}

UKF::~UKF() {}

void UKF::InitializeMeasurement(MeasurementPackage meas_package) {
  cout << "InitializeMeasurement starts" << endl;
  // initialize variables
  x_ << 1, 1, 0, 0, 0;

  // for LIDAR data
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    double px = meas_package.raw_measurements_(0);
    double py = meas_package.raw_measurements_(1);

    x_ << px, py, 0, 0, 0;
  }
  // for RADAR data
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    double rho = meas_package.raw_measurements_(0);
    double phi = meas_package.raw_measurements_(1);
    double rho_dot = meas_package.raw_measurements_(2);

    double px = rho * cos(phi);
    double py = rho * sin(phi);

    x_ << px, py, 0, 0, 0;
  }

  P_ << 0.01, 0., 0., 0., 0.,
        0., 0.01, 0., 0., 0.,
        0., 0., 1.0, 0., 0.,
        0., 0., 0., 0.01, 0.,
        0., 0., 0., 0., 0.01;

  // initialize previous time
  previous_timestamp_ = meas_package.timestamp_;

  is_initialized_ = true;
}
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
    InitializeMeasurement(meas_package);
    return;
  }

  cout << "ProcessMeasurement starts" << endl;
  // prediction
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; // seconds
  previous_timestamp_ = meas_package.timestamp_;

  // Prediction(delta_t);

  while (delta_t > 0.1)
  {
    const double dt = 0.05;
    Prediction(dt);
    delta_t -= dt;
  }
  Prediction(delta_t);

  // update
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
  }
  cout << "ProcessMeasurement ends" << endl;
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

  cout << "Prediction starts" << endl;
  ///////////////////////////
  // generate sigma points
  ///////////////////////////

  // create augmented state vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.fill(0.0);

  // matrix Q
  MatrixXd Q = MatrixXd::Zero(2,2);
  Q << std_a_ * std_a_, 0,
       0, std_yawdd_ * std_yawdd_;

  P_aug.topLeftCorner(5,5) = P_;
  P_aug.bottomRightCorner(2,2) = Q;

  // Take matrix square root
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
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
      px_pred = px + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
      py_pred = py + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    }
    else {
      px_pred = px + v * delta_t * cos(yaw);
      py_pred = py + v * delta_t * sin(yaw);
    }

    double v_pred = v;
    double yaw_pred = yaw + yawd * delta_t;
    double yawd_pred = yawd;

    // add noise
    px_pred += 0.5 * nu_a * (delta_t * delta_t) * cos(yaw);
    py_pred += 0.5 * nu_a * (delta_t * delta_t) * sin(yaw);
    v_pred += nu_a * delta_t;
    yaw_pred += 0.5 * nu_yawdd * (delta_t * delta_t);
    yawd_pred += nu_yawdd * delta_t;

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
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0); //x_;

    // normalize angle of yaw
    x_diff(3) = tools.NormalizeAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
  cout << "Prediction ends" << endl;
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

  cout << "UpdateLidar starts" << endl;
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd H_laser = MatrixXd::Zero(2, 5);
  H_laser << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0;

  // measurement noise matrix - laser
  MatrixXd R_laser = MatrixXd::Zero(2, 2);
  R_laser << std_laspx_ * std_laspx_, 0,
             0, std_laspy_ * std_laspy_;

  // calculate kalman gain K
  VectorXd z_pred = H_laser * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_laser.transpose();
  MatrixXd S = H_laser * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // Update new state of x_ and P_
  x_ += (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_laser) * P_;
  
  NIS_laser_ = (meas_package.raw_measurements_ - z_pred).transpose() * S.inverse() * (meas_package.raw_measurements_ - z_pred);
  cout << "laser NIS = " << NIS_laser_ << endl;

  cout << "UpdateLidar ends" << endl;
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

  cout << "UpdateRadar starts" << endl;
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  /////////////////////
   //  Predict
   ////////////////////
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    double px  = Xsig_pred_(0,i);
    double py  = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double vx = cos(yaw)*v;
    double vy = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(px * px + py * py);                      //rho
    Zsig(1,i) = atan2(py,px);                                 //phi
    Zsig(2,i) = (px * vx + py * vy ) / sqrt(px * px + py * py);   //rho_dot
  }

  // mean of predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
      z_pred += weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - Zsig.col(0); //z_pred;

    // normalize angle - phi
    z_diff(1) = tools.NormalizeAngle(z_diff(1));

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd::Zero(n_z,n_z);
  R <<    std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;
  S += R;

  /////////////////////
  //  Update
  ////////////////////

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd::Zero(n_z);
  z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // residual
    VectorXd z_diff = Zsig.col(i) - Zsig.col(0); //z_pred;
    //angle normalization
    z_diff(1) = tools.NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0); //x_;

    // angle normalization
    x_diff(3) = tools.NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // normalize angle
  z_diff(1) = tools.NormalizeAngle(z_diff(1));

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  cout << "radar NIS = " << NIS_radar_ << endl;

  cout << "UpdateRadar ends" << endl;
}
