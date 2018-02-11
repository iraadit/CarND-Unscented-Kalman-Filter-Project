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
  std_a_ = 30; // TO MODIFY

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30; // TO MODIFY

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

  // State dimension
  n_x_ = x_.size();
  // Augmented state dimension
  n_aug_ = n_x_ + 2;
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  // Sigma dmension
  n_sig_ = 2 * n_aug_ + 1;

  // set weights
  weights_ = VectorXd(n_sig_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < n_sig_; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_,  0,                        0,
              0,                    std_radphi_*std_radphi_,  0,
              0,                    0,                        std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_*std_laspx_,  0,
              0,                      std_laspy_*std_laspy_;

  // Set to calculate and output radar NIS
  output_NIS = true;

  // Open NIS files
  radar_NIS_file_.open("../NIS/Radar_NIS.txt", ios::out);
  if(!radar_NIS_file_.is_open()){
    cout << "Error opening Radar_NIS.txt" << endl;
    exit(1);
  }
  lidar_NIS_file_.open("../NIS/Lidar_NIS.txt", ios::out);
  if(!lidar_NIS_file_.is_open()){
    cout << "Error opening Lidar_NIS.txt" << endl;
    exit(1);
  }

  is_initialized_ = false;
}

UKF::~UKF() {
  radar_NIS_file_.close();
  lidar_NIS_file_.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // Initial covariance matrix
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    double px, py;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      px = rho * cos(phi);
      py = rho * sin(phi);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
    }
    x_ << px,
          py,
          0,
          0,
          0;

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
  * Prediction
  */
  double delta_t = (meas_package.timestamp_ - time_us_) / (double)CLOCKS_PER_SEC;
  time_us_ = meas_package.timestamp_;

  if (delta_t > .001) {
    Prediction(delta_t);
  }

/*
  std::cout << "Predicted state" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_ << std::endl;
*/

  /**
  * Update
  */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR &&  use_radar_) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
}

/**
 * Angle normalization to [-Pi, Pi]
 * @param {double} angle the angle to normalize.
 */
void UKF::NormAngle(double *angle) {
    while (*angle > M_PI) *angle -= 2. * M_PI;
    while (*angle < -M_PI) *angle += 2. * M_PI;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  /**
  * Calculate Augmented Sigma Points
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  Xsig_aug.col(0)  = x_aug;
  double sqrt_lambda_n_aug = sqrt(lambda_+n_aug_);
  for (int i = 0; i < n_aug_; i++)
  {
    VectorXd sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
    Xsig_aug.col(i+1)        = x_aug + sqrt_lambda_n_aug_L;
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug_L;
  }


  /**
  * Sigma Points Prediction
  */
  //predict sigma points
  for (int i = 0; i < n_sig_; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }


  /**
  * Predicted Mean and covariance
  */
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormAngle(&(x_diff(3)));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Represent the lidar prediction in the coordinate system as the measurement and call the update.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  Output the Lidar NIS if desired.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;
  // Transform sigma points into measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
  // Update step
  double NIS = UpdateUKF(meas_package, Zsig, n_z);

  if (output_NIS){
    lidar_NIS_file_ << NIS << endl;
  }
}

/**
 * Represent the radar prediction in the coordinate system as the measurement and call the update.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  Output the Radar NIS if desired.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  // Update step
  double NIS = UpdateUKF(meas_package, Zsig, n_z);

  if (output_NIS){
    radar_NIS_file_ << NIS << endl;
  }
}

/**
 * Updates the state and the state covariance matrix using radar or lidar measurement.
 * @param {MeasurementPackage} meas_package
 * @param Zsig The measurement sigma matrix
 * @param n_z The measurement dimension
 * @output double return the NIS
 */
double UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z){
  /**
  Use data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  Calculate the NIS.
  */
  /**
  * Predict Measurement
  */
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    NormAngle(&(z_diff(1)));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    R = R_radar_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    R = R_laser_;
  }
  S = S + R;

  /**
  * UKF Update
  */
  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 sigma points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormAngle(&(z_diff(1)));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormAngle(&(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Measurements
  VectorXd z = meas_package.raw_measurements_;

  //residual
  VectorXd z_diff = z - z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
    // Angle normalization
    NormAngle(&(z_diff(1)));
  }

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //calculate the NIS
  double NIS = z.transpose() * S.inverse() * z;

  return NIS;
}
