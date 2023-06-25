#include "ukf.h"
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// notes: CTRV
// constant turn rate and velocity
// Although the cars have variable acceleration and turning rates the UKF is still able to track the objects quite well with the CTRV model.
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.6;     // was 30
  std_yawdd_ = 1.3; // was 30
  // Process noise standard deviation yaw acceleration in rad/s^2
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

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // is state vector initialized?
  is_initialized_ = false;
  // state dimension
  n_x_ = 5;
  // initial sigma point matrix / initialize it
  // time at beggining
  time_us_ = 0;
  n_aug_ = n_x_ + 2;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);
  // define spreading parameter
  lambda_ = 3 - n_aug_;
  // Weights of sigma points
  delta_t_ = 0;
  weights_ = VectorXd(2 * n_aug_ + 1);
  // set vector for weights

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  ;

  for (int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
  //NIS_lidar_ = 0.0;
  //NIS_radar_ = 0.0;
}
UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px = meas_package.raw_measurements_[0];     // position of cyclist in x dimension
      double py = meas_package.raw_measurements_[1];     // position of cyclist in y dimension
      x_ << px, py, 0, 0, 0;
      /*[σpx^2,σpy^2,σv^2,σψ^2,σψ˙^2]*/
      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
          0, std_laspy_ * std_laspy_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_[0];     // range
      double phi = meas_package.raw_measurements_[1];     // bearing
      double rho_dot = meas_package.raw_measurements_[2]; // velocity of rho

      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);
      x_ << px, py, abs(v), 0, 0;
      /*[σpx^2,σpy^2,σv^2,σψ^2,σψ˙^2]*/
      P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
          0, std_radr_ * std_radr_, 0, 0, 0,
          0, 0, std_radrd_ * std_radrd_, 0, 0,
          0, 0, 0, std_radphi_ * std_radphi_, 0,
          0, 0, 0, 0, 1;
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true)
  {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true)
  {
    UpdateLidar(meas_package);
  }
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);             // augmented sigma matrix
  UKF::AugmentedSigmaPoints(Xsig_aug);                              // creating augmented sigma points
  UKF::SigmaPointPrediction(Xsig_aug, delta_t);                     // sigma point prediction
  UKF::PredictMeanAndCovariance();                                  // predicting mean and covariance
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /*
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  //-------------------------------------------------------------------------------
  // set vector for weights
  // create matrix for sigma points in measurement space
  // measurement covariance matrix S
  MatrixXd S_ = MatrixXd(2, 2);
  S_.fill(0.0);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(2);
  z_pred.fill(0.0);
  MatrixXd R = MatrixXd(2, 2);
  R.fill(0.0);
  double noise_ax;
  double noise_ay;
  // set the acceleration noise components
  noise_ax = std_laspx_;
  noise_ay = std_laspy_;

  R << noise_ax * noise_ax, 0,
      0, noise_ay * noise_ay;
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    float px = Xsig_pred_(0, i);
    float py = Xsig_pred_(1, i);
    Zsig.col(i) << px, py;
    z_pred = z_pred + (weights_(i) * Zsig.col(i));
  }

  for (int j = 0; j < 2 * n_aug_ + 1; j++)
  {
    VectorXd z_diff = Zsig.col(j) - z_pred;
    S_ = S_ + (weights_(j) * z_diff * z_diff.transpose());
  }
  S_ = S_ + R;

  //-------------------------------------------------------------------------------
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 2);
  Tc.fill(0.0);
  /**
   * Student part begin
   */
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd z_ = VectorXd(2);
  z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

  // calculate cross correlation matrix
  for (int k = 0; k < 2 * n_aug_ + 1; k++)
  {
    VectorXd z_diff = Zsig.col(k) - z_pred;

    VectorXd x_diff = Xsig_pred_.col(k) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    Tc = Tc + (weights_(k) * x_diff * z_diff.transpose());
  }
  VectorXd z_diff = z_ - z_pred;
  // calculate Kalman gain K;
  MatrixXd kalman_gain = Tc * S_.inverse();
  // update state mean and covariance matrix
  x_ = x_ + (kalman_gain * z_diff);
  P_ = P_ - (kalman_gain * S_ * kalman_gain.transpose());

  NIS_lidar_ = z_diff.transpose() * S_.inverse() * z_diff;
  //std::cout << "NIS Lidar: " << NIS_lidar_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /*
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  //-------------------------------------------------------------------------------
  // set vector for weights
  // create matrix for sigma points in measurement space
  // measurement covariance matrix S
  MatrixXd S_ = MatrixXd(3, 3);
  S_.fill(0.0);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  MatrixXd R = MatrixXd(3, 3);
  R.fill(0.0);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  // convert Predicted Sigma points to radar measurement state (Zsig)
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double vx = v * cos(yaw);
    double vy = v * sin(yaw);

    double rho = std::sqrt((px * px) + (py * py)); // range
    double phi = atan2(py, px);                    // angle 
    double rho_dot = ((px * vx) + (py * vy)) / rho; // velocity of rho

    Zsig.col(i) << rho,
        phi,
        rho_dot;
    // fill in predicted state of X in Z state (measurement)
    z_pred = z_pred + (weights_(i) * Zsig.col(i));
  }

  for (int j = 0; j < 2 * n_aug_ + 1; j++)
  {
    VectorXd z_diff = Zsig.col(j) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    S_ = S_ + (weights_(j) * z_diff * z_diff.transpose());
  }
  S_ = S_ + R;

  //-------------------------------------------------------------------------------
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc.fill(0.0);
  /**
   * Student part begin
   */
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd z_ = VectorXd(3);
  z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

  // calculate cross correlation matrix
  for (int k = 0; k < 2 * n_aug_ + 1; k++)
  {
    VectorXd z_diff = Zsig.col(k) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    VectorXd x_diff = Xsig_pred_.col(k) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    Tc = Tc + weights_(k) * x_diff * z_diff.transpose();
  }
  // calculate Kalman gain K;
  VectorXd z_diff = z_ - z_pred;
  // Normalize angle
  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  // MatrixXd kalman_gain = MatrixXd(n_x_, 3);
  MatrixXd kalman_gain = Tc * S_.inverse();
  // update state mean and covariance matrix
  x_ = x_ + (kalman_gain * z_diff);
  P_ = P_ - (kalman_gain * S_ * kalman_gain.transpose());

  NIS_radar_ = z_diff.transpose() * S_.inverse() * z_diff;
  //std::cout << "NIS Radar: " << NIS_radar_ << std::endl;
}

// function not used since we need to add noise to our sigma points
void UKF::GenerateSigmaPoints(Eigen::MatrixXd *Xsig_out)
{
  float scalar = std::sqrt(3);
  // calculate square root of P
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  MatrixXd temp = MatrixXd(n_x_, n_x_);
  int i = 1;
  MatrixXd A = P_.llt().matrixL();
  Xsig.col(0) = x_;
  // generate sigma points
  while (i < Xsig.cols())
  {
    MatrixXd temp = (A * scalar);
    if (i == 1)
    {
      for (int j = 0; j < temp.cols(); j++)
      {
        Xsig.col(i++) = x_ + temp.col(j);
      }
    }
    else if (i > 5)
    {
      for (int j = 0; j < temp.cols(); j++)
      {
        Xsig.col(i++) = x_ - temp.col(j);
      }
    }
  }
  // write result
  *Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug)
{
  Xsig_aug.fill(0.0);
  /* Sigma point Augmentation */
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug << x_[0], x_[1], x_[2], x_[3], x_[4], 0, 0;
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.block<5, 5>(0, 0) = P_;
  MatrixXd Q = MatrixXd(2, 2);
  Q.fill(0.0);
  Q << std_a_ * std_a_, 0, 0, std_yawdd_ * std_yawdd_;
  P_aug.block<2, 2>(5, 5) = Q;
  // create sigma point matrix
  MatrixXd A = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + (std::sqrt(lambda_ + n_aug_) * A.col(i));
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - (std::sqrt(lambda_ + n_aug_) * A.col(i));
  }
}

void UKF::PredictMeanAndCovariance()
{
  /* Predict Mean and Covariance */
  // create vector for predicted state
  VectorXd x_predicted = VectorXd(n_x_);
  x_predicted.fill(0.0);
  // create covariance matrix for Prediction
  MatrixXd P_predicted = MatrixXd(n_x_, n_x_);
  P_predicted.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_predicted = x_predicted + (weights_[i] * Xsig_pred_.col(i));
  }

  for (int j = 0; j < 2 * n_aug_ + 1; j++)
  {
    VectorXd x_diff = Xsig_pred_.col(j) - x_predicted;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    P_predicted = P_predicted + (weights_[j] * x_diff * x_diff.transpose());
  }
  x_.fill(0.0);
  P_.fill(0.0);
  x_ = x_predicted;
  P_ = P_predicted;
}

void UKF::SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, double delta_t)
{
  /**
   * Student part begin
   */
  // Read values from current state vector
  VectorXd state_prt0 = VectorXd(n_x_);
  VectorXd state_prt1 = VectorXd(n_x_);
  VectorXd state_prt2 = VectorXd(n_x_);
  state_prt0.fill(0.0);
  state_prt1.fill(0.0);
  state_prt2.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    state_prt0 << Xsig_aug(0, i), Xsig_aug(1, i), Xsig_aug(2, i), Xsig_aug(3, i), Xsig_aug(4, i);
    if (fabs(Xsig_aug(4, i)) > 0.001)
    {
      state_prt1 << (Xsig_aug(2, i) / Xsig_aug(4, i)) * (sin(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) - sin(Xsig_aug(3, i))),
          (Xsig_aug(2, i) / Xsig_aug(4, i)) * (-cos(Xsig_aug(3, i) + Xsig_aug(4, i) * delta_t) + cos(Xsig_aug(3, i))),
          (0),
          (Xsig_aug(4, i) * delta_t),
          (0);
    }
    else
    {
      state_prt1 << Xsig_aug(2, i) * cos(Xsig_aug(3, i)) * delta_t,
          Xsig_aug(2, i) * sin(Xsig_aug(3, i)) * delta_t,
          (0),
          (Xsig_aug(4, i) * delta_t),
          (0);
    }
    state_prt2 << (0.5) * ((delta_t * delta_t) * cos(Xsig_aug(3, i)) * Xsig_aug(5, i)),
        (0.5) * ((delta_t * delta_t) * sin(Xsig_aug(3, i)) * Xsig_aug(5, i)),
        (delta_t * Xsig_aug(5, i)),
        (0.5) * ((delta_t * delta_t) * Xsig_aug(6, i)),
        (delta_t * Xsig_aug(6, i));
    Xsig_pred_.col(i) = state_prt0 + state_prt1 + state_prt2;
  }
}