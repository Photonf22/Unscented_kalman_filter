#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include "ukf.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;


// notes: CTRV
// constant turn rate and velocity
// Although the cars have variable acceleration and turning rates the UKF is still able to track the objects quite well with the CTRV model.
/**
 * Initializes Unscented Kalman filter
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
  std_a_ = 0.11;     // was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.11; // was 30
  
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
  n_z = 3;
  // initial sigma point matrix / initialize it
  // time at beggining
  time_us_ = 0;
  Xsig_pred_ = MatrixXd(n_x_, n_aug_);
  n_aug_= 2*n_x_ + 1;
    // define spreading parameter
  lambda_ = 3 - n_x_;
    // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  z_ << std_radr_,std_radphi_,std_radrd_;
    // create matrix for sigma points in measurement space
  Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  delta_t_ = 0;
  z_pred = VectorXd(n_z);
}


UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  
  double Average_time = 0;
  if(is_initialized_ == false)
  {
    time_us_= meas_package.timestamp_;
    is_initialized_ = true;
     
  }
  else
  {
    delta_t_ = (meas_package.timestamp_ - time_us_)/ 1000000.0; 
    time_us_ = meas_package.timestamp_;
    measurement_type = meas_package.sensor_type_;
      // Initialize state and covariance matrix
    if(meas_package.sensor_type_ == 0)
    {
        // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      P_ <<      0.10,    0,    0,    0,    0,
                 0,    0.10,    0,    0,    0,
                 0,    0,    0.05,    0,    0,
                 0,    0,    0,    0.05,    0,
                 0,    0,    0,    0,    0.05;
      std::cout << "im here 0" << std::endl;
      //Prediction(delta_t_);
      //UpdateLidar(meas_package) ;
    }
    else if(meas_package.sensor_type_ == 1)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      P_ <<   0.10,    0,    0,    0,    0,
                 0,    0.10,    0,    0,    0,
                 0,    0,    0.05,    0,    0,
                 0,    0,    0,    0.05,    0,
                 0,    0,    0,    0,    0.05;
      std::cout << "im here 1" << std::endl;
      //Prediction(delta_t_);
      //UpdateRadar(meas_package) ;
    }
  }
  
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
  */
 std::cout << "im here 2" << std::endl;
/*
 // set measurement dimension, radar can measure r, phi, and r_dot
  MatrixXd Xsig = MatrixXd(n_x_, n_aug_);
  // create covariance matrix for Prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  UKF::GenerateSigmaPoints(&Xsig);
  UKF::AugmentedSigmaPoints(&Xsig);
  UKF::SigmaPointPrediction(&Xsig_pred_, Xsig , delta_t);
  
  if(measurement_type== 0)
    {
      UKF::PredictMeanAndCovarianceLaser(&x_, &P_);
      
    }
    else if(measurement_type == 1)
    {
      UKF::PredictRadarMeasurement(&z_pred,&S);
    }
    */
}

void UKF::UpdateLidar(MeasurementPackage meas_package ) 
{
  /*
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
        //acceleration noise components
  double noise_ax;
  double noise_ay;
  // set the acceleration noise components
  noise_ax = std_laspx_;
  noise_ay = std_laspy_;

  // measurement covariance
  R_ = MatrixXd(n_x_, n_x_);
  R_ << noise_ax, 0,
            0, noise_ay;

  // measurement matrix
  H_ = MatrixXd(2, n_x_);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  /*
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

   // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  /**
   * Student part begin
   */

  // calculate cross correlation matrix
  for(int i = 0;i < 2*n_aug_+1;i++)
  {
    Tc = Tc + (weights_(i)*(Xsig_pred_.col(i) - x_)*(Zsig.col(i) - z_pred).transpose());
  }
  
  // calculate Kalman gain K;
  MatrixXd kalman_gain = MatrixXd(n_x_, n_z);
  kalman_gain =  Tc * S.inverse();
  // update state mean and covariance matrix
  x_ = x_ + kalman_gain*(z_ - z_pred);
  P_ = P_ - kalman_gain*S*kalman_gain.transpose();
  /**
   * Student part end
   */

  // print result
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  
}

void UKF::GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out) {
  float   scalar = std::sqrt(3);
  // calculate square root of P
  MatrixXd Xsig = MatrixXd(n_x_, n_aug_);
  MatrixXd temp = MatrixXd(n_x_,n_x_);
  int i = 1;
  MatrixXd A = P_.llt().matrixL();
  Xsig.col(0) = x_;
  //generate sigma points
  while(i < Xsig.cols())
  {
    MatrixXd temp = (A * scalar);
    if(i == 1)
    {
      for(int j = 0;j < temp.cols();j++)
      {
        Xsig.col(i++) = x_ + temp.col(j);
      }
    }
    else if(i > 5)
    {
      for(int j = 0;j < temp.cols();j++)
      {
        Xsig.col(i++) = x_ - temp.col(j);
      }
    }
  }
    // write result
  *Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out) 
{
  // create augmented mean vector
  VectorXd x_aug= VectorXd(n_aug_);
  x_aug << x_[0],x_[1],x_[2],x_[3],x_[4], 0, 0;
    // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.block<5,5>(0,0) = P_;
  VectorXd zeros = VectorXd(5);
  zeros<< 0,0,0,0,0;
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;
  
  P_aug.block<1,5>(5,0) = zeros;
  P_aug.block<1,5>(6,0) = zeros;
  P_aug.block<2,2>(5,5) = Q;
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MatrixXd A =  P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  for(int i = 0;i < n_aug_;i++)
  {
    Xsig_aug.col(i+1) = x_aug + + std::sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - + std::sqrt(lambda_+n_aug_) * A.col(i);
  }
  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(Eigen::MatrixXd* Xsig_out, Eigen::MatrixXd & Xsig_aug , double delta_t) 
{
  /**
   * Student part begin
   */
  // predict sigma points
    VectorXd state_prt0 =  VectorXd(5);
    VectorXd state_prt1 = VectorXd(5);
    VectorXd state_prt2 = VectorXd(5);
  for(int i = 0 ;i < (2 * n_aug_ + 1);i++)
  {
    state_prt0 <<  Xsig_aug(0,i),Xsig_aug(1,i),Xsig_aug(2,i),Xsig_aug(3,i),Xsig_aug(4,i);
    
    if(Xsig_aug(4,i) == 0)
    {
      state_prt1<< Xsig_aug(2,i)*cos(Xsig_aug(3,i))*delta_t,Xsig_aug(2,i)*sin(Xsig_aug(3,i))*delta_t,(0),(Xsig_aug(4,i)*delta_t),(0);
      state_prt2<< (0.5)*((delta_t*delta_t)*cos(Xsig_aug(3,i))*Xsig_aug(5,i)),(0.5)*((delta_t*delta_t)*sin(Xsig_aug(3,i))*Xsig_aug(5,i)),(delta_t*Xsig_aug(5,i)),(1/2)*((delta_t*delta_t)*Xsig_aug(6,i)),(delta_t*Xsig_aug(6,i));
    }
    else
    {
      state_prt1<< (Xsig_aug(2,i)/Xsig_aug(4,i)) * (sin(Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t) - sin(Xsig_aug(3,i))),(Xsig_aug(2,i)/Xsig_aug(4,i)) * (-cos(Xsig_aug(3,i) + Xsig_aug(4,i)*delta_t) + cos(Xsig_aug(3,i))),(0),(Xsig_aug(4,i)*delta_t),(0);
      state_prt2<< (0.5)*((delta_t*delta_t)*cos(Xsig_aug(3,i))*Xsig_aug(5,i)),(0.5)*((delta_t*delta_t)*sin(Xsig_aug(3,i))*Xsig_aug(5,i)),(delta_t*Xsig_aug(5,i)),(0.5)*((delta_t*delta_t)*Xsig_aug(6,i)),(delta_t*Xsig_aug(6,i));
    }
 
    Xsig_pred_.col(i) =  state_prt0 + state_prt1 + state_prt2;
  }
}

void UKF::PredictMeanAndCovarianceLaser(Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out) {
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for Prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  /**
   * Student part begin
   */

  // set weights
  for(int i = 0;i < Xsig_pred_.cols();i++)
  {
      if(i == 0){
        weights_[i]= lambda_/ (lambda_ + n_aug_); 
      }
      else
      {
          weights_[i]= 1/ (2*(lambda_ + n_aug_)); 
      }
  }
  float result= 0.0;
  // predict state mean
  for(int j = 0; j < 2*n_aug_+1;j++)
  {
    x = x + (weights_[j] * Xsig_pred_.col(j));
  }
  // predict state covariance matrix
   for(int j = 0; j < 2*n_aug_+1;j++)
  {
    P = P + (weights_[j] * ((Xsig_pred_.col(j) - x)*(Xsig_pred_.col(j) - x).transpose()));
  }
  /**
   * Student part end
   */

  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  // write result
  *x_out = x;
  *P_out = P;
}
/**
 * Programming assignment functions: 
 */

void UKF::PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out) {

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; ++i) {  
    weights(i) = weight;
  }

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  /**
   * Student part begin
   */
 VectorXd temp = VectorXd(3);
  // transform sigma points into measurement space
  // measurement model
  for(int i = 0 ;i < Xsig_pred_.cols(); i++)
  {
      temp << std::sqrt((Xsig_pred_(0,i)* Xsig_pred_(0,i)) + (Xsig_pred_(1,i)*Xsig_pred_(1,i))),atan(Xsig_pred_(1,i)/Xsig_pred_(0,i)),((Xsig_pred_(0,i)*cos(Xsig_pred_(3,i))*Xsig_pred_(2,i)) + (Xsig_pred_(1,i)*sin(Xsig_pred_(3,i))*Xsig_pred_(2,i)))/std::sqrt((Xsig_pred_(0,i) * Xsig_pred_(0,i))+(Xsig_pred_(1,i) * Xsig_pred_(1,i))) ;
      Zsig.col(i) = temp;
  }
  // calculate mean predicted measurement
  for(int j = 0;j < 2 * n_aug_ + 1;j++)
  {
    z_pred = z_pred + (weights(j) *Zsig.col(j)); 
  }
  // calculate innovation covariance matrix S
  MatrixXd R =  MatrixXd(3,3);
  VectorXd temp2 = VectorXd(n_z);
  R << std_radr_*std_radr_,0,0,0,std_radphi_*std_radphi_,0,0,0,std_radrd_*std_radrd_;
  for(int k = 0;k < 2 * n_aug_ + 1;k++) 
  {
    S = (S +  weights(k) *(Zsig.col(k) - z_pred)*(Zsig.col(k) - z_pred).transpose());
  }
  S = S + R;
  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}