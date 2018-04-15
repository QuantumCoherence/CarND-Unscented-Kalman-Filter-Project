#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
//#define PSIERROROUT
//#define XSIGPRINT

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
	is_initialized_ = false;
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// state vector dimension
	n_x_ = 5;
	// radar measurement  vector dimension
	n_z_ldr_ = 2;
	n_z_rdr_ = 3;
	lambda_= 3;
	// augment state vector dimension
	n_aug_ = 7;

	// initial state vector
	x_ = VectorXd(n_x_);

	// initial covariance matrix
	P_ = MatrixXd(n_x_, n_x_);
	P_ << 1, 0, 0, 0, 0,
		  0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
		  0, 0, 0, 0, 1;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 0.030;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 0.030;

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

	// Create Sigma Point Matrix
	Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);


	//create augmented mean vector
	x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	P_aug = MatrixXd(n_aug_, n_aug_);

	//create sigma point matrix
	Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	//create predicted sigma point matrix
	Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

	//create vector for weights
	weights_ = VectorXd(2 * n_aug_ +1);
    //set weights
    weights_(0) = lambda_/(lambda_ + n_aug_);
    for (int i=1; i< 2 * n_aug_ +1;i++){
	   weights_(i) = 1/(2*(lambda_ + n_aug_));
	}

	//create matrix for sigma points in measurement space
	Zsig_rdr = MatrixXd(n_z_rdr_, 2 * n_aug_ + 1);
	Zsig_ldr = MatrixXd(n_z_ldr_, 2 * n_aug_ + 1);

	//Radar mean predicted measurement
	z_pred_rdr_ = VectorXd(n_z_rdr_);
	z_pred_ldr_ = VectorXd(n_z_ldr_);

	// Radar measurement covariance matrix S
	S_rdr_ = MatrixXd(n_z_rdr_,n_z_rdr_);
	//Lidar measurement covariance matrix S
	S_ldr_ = MatrixXd(n_z_ldr_,n_z_ldr_);

	//create matrix for cross correlation Tc
	Tc_rdr = MatrixXd(n_x_, n_z_rdr_);
	Tc_ldr = MatrixXd(n_x_, n_z_ldr_);

	z_rdr = VectorXd(n_z_rdr_);
	  /*
	  *    z <<
	  *    5.9214,   //rho in m
	  *    0.2187,   //phi in rad
	  *    2.0062;   //rho_dot in m/s
	  */
	  //create vector for incoming radar measurement
	  z_ldr = VectorXd(n_z_ldr_);
	  /*
	  *    z <<
	  *    5.9214,   //px in m
	  *    0.2187,   //py in m
	  */

	  nis_ = 0.0;
	  // lambda factor
	  lambda_fx_aug = sqrt(lambda_+n_aug_);
	  A_aug = MatrixXd(n_aug_,n_aug_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) {

	  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		  x_ << meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]), meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]), 0, 0.1, 0.1;
	  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		  x_ << 3.122427e-01,	5.803398e-01,	5	,0, 0.0;
		  //this->x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 5	,0;
	  }

	  // done initializing, no need to predict or update
	  previous_timestamp_ = meas_package.timestamp_;
	  is_initialized_ = true;
	  return;
	}
	if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) return;
	if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) return;
	//compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;


	Prediction(dt);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	// Radar updates
		//cout << "Radar " << meas_package.raw_measurements_ << endl;
		UpdateRadar(meas_package);
	} else {
	// Laser updates
		//cout << "Lidar " << meas_package.raw_measurements_ << endl;
		UpdateLidar(meas_package);
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

	int i;

	//create augmented mean state
	x_aug.head(5) = x_;
	x_aug[5] = 0.0;
	x_aug[6] = 0.0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5,5) = P_;
	P_aug(5,5)  =  std_a_ * std_a_;
	P_aug(6,6)  =  std_yawdd_ * std_yawdd_;

	//calculate square root of P
	A_aug = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0)  = x_aug;
	for (i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i+1)       = x_aug + lambda_fx_aug * A_aug.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - lambda_fx_aug * A_aug.col(i);
	}


	//Sigma Points Prediction
	for (i=0; i<2*n_aug_+1;i++){
	      double cospsik  = cos(Xsig_aug(3, i));
	      double sinpsik  = sin(Xsig_aug(3, i));
	      double psidkdt  = Xsig_aug(4, i)*delta_t;
	      double psidk    = Xsig_aug(4, i);
	      double psik     = Xsig_aug(3, i);
	      double vk       = Xsig_aug(2, i);
	      double nuak     = Xsig_aug(5, i);
	      double nupsiddk = Xsig_aug(6, i);
	      double halfdt2  = 0.5*delta_t*delta_t;
	      if ( fabs(psidk) > 0.001) {
	          Xsig_pred(0, i) =  Xsig_aug(0, i) + vk/psidk*(sin(psik+psidkdt) - sinpsik) + halfdt2*cospsik*nuak;
	          Xsig_pred(1, i) =  Xsig_aug(1, i) + vk/psidk*(-cos(psik+psidkdt)+ cospsik) + halfdt2*sinpsik*nuak;
	          Xsig_pred(2, i) =  Xsig_aug(2, i) + delta_t*nuak;
	          Xsig_pred(3, i) =  Xsig_aug(3, i) + psidkdt + 0.5 * delta_t*delta_t*nupsiddk;
	          Xsig_pred(4, i) =  Xsig_aug(4, i) + delta_t*nupsiddk;
	      } else {
	          Xsig_pred(0, i) =  Xsig_aug(0, i) + vk*cospsik*delta_t + halfdt2*cospsik*nuak;
	          Xsig_pred(1, i) =  Xsig_aug(1, i) + vk*sinpsik*delta_t + halfdt2*sinpsik*nuak;
	          Xsig_pred(2, i) =  Xsig_aug(2, i) + delta_t*nuak;
	          Xsig_pred(3, i) =  Xsig_aug(3, i) + 0.5 * delta_t*delta_t*nupsiddk;
	          Xsig_pred(4, i) =  Xsig_aug(4, i) + delta_t*nupsiddk;
	      }
	  }


	  //predict state mean
	  x_.fill(0.0);
	  for (i=0;i<2*n_aug_ +1;i++){
	      x_  = x_ + weights_(i)* Xsig_pred.col(i);
	  }
	  //predict state covariance matrix

	  VectorXd sig_err = VectorXd(n_x_);
	  P_.fill(0.0);
	  for (i=0;i<2*n_aug_ +1;i++){
		  // state difference
	      sig_err = (Xsig_pred.col(i)-x_);
	      while (sig_err(3)> M_PI) sig_err(3)-=2.*M_PI;
	      while (sig_err(3)<-M_PI) sig_err(3)+=2.*M_PI;
	      P_ = P_ + weights_(i)*sig_err*sig_err.transpose();
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

	int i;
	//transform sigma points into measurement space
	for(i=0;i<2*n_aug_+1;i++){
	  Zsig_ldr(0, i) = Xsig_pred(0,i);
	  Zsig_ldr(1, i) = Xsig_pred(1,i);
	}
	//calculate mean predicted measurement
	z_pred_ldr_.fill(0.0);
	z_pred_ldr_ = z_pred_ldr_ + weights_(0)*Zsig_ldr.col(0);
	for(i=1;i<2*n_aug_+1;i++){
	  z_pred_ldr_ = z_pred_ldr_ + weights_(i)* Zsig_ldr.col(i);
	}

	//calculate innovation covariance matrix S
	S_ldr_.fill(0.0);
	VectorXd S_err = VectorXd(n_z_ldr_);
	S_err = (Zsig_ldr.col(0)-z_pred_ldr_);

	S_ldr_ = weights_(0) * S_err * S_err.transpose();
	for (i=1;i<2*n_aug_+1;i++){
	  S_err = (Zsig_ldr.col(i)-z_pred_ldr_);
      S_ldr_ = S_ldr_ + weights_(i)*S_err*S_err.transpose();
	}
	S_ldr_(0,0) = S_ldr_(0,0) + std_laspx_ * std_laspx_;
	S_ldr_(1,1) = S_ldr_(1,1) + std_laspy_ * std_laspy_; // + R

	z_ldr = meas_package.raw_measurements_;
	//calculate cross correlation matrix
	VectorXd X_err = VectorXd(n_x_);
	VectorXd Z_err = VectorXd(n_z_ldr_);
	MatrixXd K = MatrixXd(n_x_, n_z_ldr_);
	Tc_ldr.fill(0.0);

	X_err = (Xsig_pred.col(0)-x_ );
	//angle normalization
    while (X_err(3)> M_PI) X_err(3)-=2.*M_PI;
    while (X_err(3)<-M_PI) X_err(3)+=2.*M_PI;
	Z_err = (Zsig_ldr.col(0)-z_pred_ldr_);


	Tc_ldr = weights_(0) * X_err * Z_err.transpose();
	for (i=1;i<2*n_aug_+1;i++){
		X_err = (Xsig_pred.col(i)-x_ );
	    //angle normalization
	    while (X_err(3)> M_PI) X_err(3)-=2.*M_PI;
	    while (X_err(3)<-M_PI) X_err(3)+=2.*M_PI;
		Z_err = (Zsig_ldr.col(i)-z_pred_ldr_);
		Tc_ldr = Tc_ldr + weights_(i)* X_err * Z_err.transpose();
	}
	//calculate Kalman gain K;
	K = Tc_ldr * S_ldr_.inverse();
	//update state mean and covariance matrix
	Z_err = z_ldr - z_pred_ldr_;

	x_ = x_ + K*(Z_err);
	P_= P_ - K*S_ldr_*K.transpose();

	nis_  = Z_err.transpose() * S_ldr_.inverse() * Z_err;
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
	int i;
	//transform sigma points into measurement space
	for(i=0;i<2*n_aug_+1;i++){
	  double px  = Xsig_pred(0,i);
	  double py  = Xsig_pred(1,i);
	  double v   = Xsig_pred(2,i);
	  double psi = Xsig_pred(3,i);
	  Zsig_rdr(0, i) = sqrt(px*px + py*py);
	  Zsig_rdr(1, i) = atan2(py,px);
	  Zsig_rdr(2, i) = (px*cos(psi)+py*sin(psi))*v/Zsig_rdr(0, i);
	}


	//calculate mean predicted measurement
	z_pred_rdr_.fill(0.0);
	z_pred_rdr_ = z_pred_rdr_ + weights_(0)*Zsig_rdr.col(0);
	for(i=1;i<2*n_aug_+1;i++){
	  z_pred_rdr_ = z_pred_rdr_ + weights_(i)* Zsig_rdr.col(i);
	}

	//calculate innovation covariance matrix S
	S_rdr_.fill(0.0);
	VectorXd S_err = VectorXd(n_z_rdr_);
	S_err = (Zsig_rdr.col(0)-z_pred_rdr_);
    //angle normalization
    while (S_err(1)> M_PI) S_err(1)-=2.*M_PI;
    while (S_err(1)<-M_PI) S_err(1)+=2.*M_PI;

	S_rdr_ = weights_(0) * S_err * S_err.transpose();

	for (i=1;i<2*n_aug_+1;i++){
	  S_err = (Zsig_rdr.col(i)-z_pred_rdr_);
      while (S_err(1)> M_PI) S_err(1)-=2.*M_PI;
	  while (S_err(1)<-M_PI) S_err(1)+=2.*M_PI;
	  S_rdr_ = S_rdr_ + weights_(i)*S_err*S_err.transpose();
	}
	S_rdr_(0,0) = S_rdr_(0,0) + std_radr_ * std_radr_;
	S_rdr_(1,1) = S_rdr_(1,1) + std_radphi_ * std_radphi_; // + R
	S_rdr_(2,2) = S_rdr_(2,2) + std_radrd_ * std_radrd_;



	z_rdr = meas_package.raw_measurements_;
	//calculate cross correlation matrix
	VectorXd X_err = VectorXd(n_x_);
	VectorXd Z_err = VectorXd(n_z_rdr_);
	MatrixXd K = MatrixXd(n_x_, n_z_rdr_);
	Tc_rdr.fill(0.0);

	X_err = (Xsig_pred.col(0)-x_ );
    //angle normalization
    while (X_err(3)> M_PI) X_err(3)-=2.*M_PI;
    while (X_err(3)<-M_PI) X_err(3)+=2.*M_PI;
	Z_err = (Zsig_rdr.col(0)-z_pred_rdr_);
    //angle normalization
    while (Z_err(1)> M_PI) Z_err(1)-=2.*M_PI;
    while (Z_err(1)<-M_PI) Z_err(1)+=2.*M_PI;


	Tc_rdr = Tc_rdr + weights_(0) * X_err * Z_err.transpose();
	for (i=1;i<2*n_aug_+1;i++){
		X_err = (Xsig_pred.col(i)-x_ );
	    //angle normalization
	    while (X_err(3)> M_PI) X_err(3)-=2.*M_PI;
	    while (X_err(3)<-M_PI) X_err(3)+=2.*M_PI;
		Z_err = (Zsig_rdr.col(i)-z_pred_rdr_);
	    //angle normalization
	    while (Z_err(1)> M_PI) Z_err(1)-=2.*M_PI;
	    while (Z_err(1)<-M_PI) Z_err(1)+=2.*M_PI;

		Tc_rdr = Tc_rdr + weights_(i)* X_err * Z_err.transpose();
	}

	//calculate Kalman gain K;
	K = Tc_rdr * S_rdr_.inverse();
	//update state mean and covariance matrix
	Z_err = z_rdr-z_pred_rdr_;
	//angle normalization
	while (Z_err(1)> M_PI) Z_err(1)-=2.*M_PI;
	while (Z_err(1)<-M_PI) Z_err(1)+=2.*M_PI;

	x_ = x_ + K*(Z_err);
	P_= P_ - K*S_rdr_*K.transpose();

	nis_  = Z_err.transpose() * S_rdr_.inverse() * Z_err;

}
