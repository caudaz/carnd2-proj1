#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //#######################################  
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  //#######################################    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //#######################################  
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;
  //#######################################      
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //#######################################   
  // CONVERT THE CURRENT STATE x_ TO POLAR COORDINATES:
  // RADIUS rho
  float rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  // ANGLE phi
  float phi = atan2(x_[1], x_[0]);
  // R VELOCITY
  float rhodot;
  if (fabs(rho<0.0001)){
    rhodot = 0.0;
  }
  else{
    rhodot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;
  }
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;

  VectorXd y = z - z_pred;
  // Perform normalization when you subtract 2 angles
  // to ensure angle is in the range -pi to +pi
  // After taking the difference of the new RADAR measurement 
  // AND the state (converted to polar coordinates)
  // y(1) is rho (the angle of the RADAR measurement)
  //
  // OPTION #1 (MORE CLEAR)
  //  while (y(1)>+3.1416) { y(1) -= 2 * 3.1416; }
  //  while (y(1)<-3.1416) { y(1) += 2 * 3.1416; }
  //
  // OPTION #2 (CLEANER)
  y(1) = atan2(sin(y(1)),cos(y(1)));
 
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;  
  //#######################################       
}
