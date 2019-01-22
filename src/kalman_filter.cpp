#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_ = F_ * x_ ;
  MatrixXd F_trans = F_.transpose();
  P_ = F_ * P_ * F_trans + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd Y_ = z - H_ * x_;
  Common_Update(Y_);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float ro_dot = (px*vx + py*vy) / rho;
  
  VectorXd h_p = VectorXd(3);
  h_p << rho, theta, ro_dot;
  VectorXd Y_ = z - h_p;
  if ( Y_(1) > M_PI ) 
  {
      Y_(1) -= 2 * M_PI;
  } 
  else if ( Y_(1) < (-M_PI) ) 
  {
      Y_(1) += 2 * M_PI;
  }

  Common_Update(Y_);

}

void KalmanFilter::Common_Update(const VectorXd &Y_){

  MatrixXd H_trans = H_.transpose();
  MatrixXd S_ = H_ * P_ * H_trans + R_;
  MatrixXd S_inv = S_.inverse();
  MatrixXd K_ =  P_ * H_trans * S_inv;

  x_ = x_ + ( K_ * Y_ );
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - ( K_ * H_ ) ) * P_;

}
