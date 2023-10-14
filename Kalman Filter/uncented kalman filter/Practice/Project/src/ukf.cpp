#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // Initially set to false
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented State dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  // Predicted sigma point
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 1 / (2 * lambda_ + 2 * n_aug_);
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.0;

  // time when the state is true
  time_us_ = 0.0;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  bool bDebug = false;
  if(!is_initialized_){
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0,
            0,
            0;

      P_ << std_laspx_ * std_laspx_, 0, 0, 0,
            0, std_laspy_ * std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 1, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      x_ << rho * cos(phi),
            rho * sin(phi),
            0,
            0,
            0;
      
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
          0, std_radr_ * std_radr_, 0, 0, 0,
          0, 0, std_radrd_ * std_radrd_, 0, 0,
          0, 0, 0, std_radphi_ * std_radphi_, 0,
          0, 0, 0, 0, 1;
    }
    else{
      std::cout << "UKF::ProcessMeasurement() error: cannot initialize because of invalid measurement sensor type " << meas_package.sensor_type_<< std::endl;
      exit(EXIT_FAILURE);
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    if (bDebug){
      std::cout << "UKF::ProcessMeasurement() initializes for sensor type " << meas_package.sensor_type_ << " - state vector x:\n" << x_ << std::endl;
    }
    return;
  }

  // compute the time elapsed between the current and previous measurements dt in seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  if (bDebug)
    std::cout << "UKF::ProcessMeasurement() predicts the state vector x:\n" << x_ << std::endl;

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
    if (bDebug)
      std::cout << "UKF::ProcessMeasurement() Lidar update - state vector x:\n" << x_ << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
    if (bDebug)
      std::cout << "UKF::ProcessMeasurement() Radar update - state vector x:\n" << x_ << std::endl;
  }
  else {
    std::cerr << "UKF::ProcessMeasurement() error: cannot update measurement because of invalid measurement sensor type " << meas_package.sensor_type_ << std::endl;
    exit(EXIT_FAILURE);
  }
}

void UKF::Prediction(double delta_t) {

  /* Generate augmented sigma point */

  // Make augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Make augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Sigman Square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Make Augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++){
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /* Predict Sigman point*/

  // create matrix with predicted sigma points as columns
  // predict sigma points
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predict the state
    double px_pred, py_pred;

    // avoid division by zero
    if(fabs(yawd) > 0.001){
      px_pred = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_pred = py + v / yawd * (-1 * cos(yaw + yawd * delta_t) + cos(yaw));
    }
    else{
      px_pred = px + v * cos(yaw) * delta_t;
      py_pred = py + v * sin(yaw) * delta_t;
    }

    double v_pred = v;
    double yaw_pred = yaw + yawd * delta_t;
    double yawd_pred = yawd;

    // add noise
    px_pred += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
    py_pred += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
    v_pred += delta_t * nu_a;
    yaw_pred += 0.5 * delta_t * delta_t * nu_yawdd;
    yawd_pred += delta_t * nu_yawdd;

    //write predicted sigma points into right column
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }

  /* 3. Predict mean and covariance */

  // predict state mean
  x_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  // set measurement dimension
  int n_z = 2;

  // Matrix for sigman points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);

  //mean prediction measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Make matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean prediction measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);


  // Create vector for incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  // create matrix for cross correlation TC
  MatrixXd Tc = MatrixXd(n_x_, n_z);


  /* 1. Transform sigma points into measurement space */
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * v1 + py * v2) / sqrt(px * px + py * py);
    
  }

  /* 2. Calculate mean, covariance */

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Calculate Innovation covarince matrix S
  S.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) {
      z_diff(1) -= 2.0 * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2.0 * M_PI;
    }

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0,std_radrd_ * std_radrd_;

  S = S + R;

  /* 3. Update state mean and covariance */

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) {
      z_diff(1) -= 2.0 * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2.0 * M_PI;
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    Tc = Tc + weights_(i) * x_diff * x_diff.transpose();
  }

  // Calcualte kalman gain K
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1) > M_PI) {
    z_diff(1) -= 2.0 * M_PI;
  }
  while (z_diff(1) < -M_PI) {
    z_diff(1) += 2.0 * M_PI;
  }

  // Update State MEAN & COVARIANCE matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
}