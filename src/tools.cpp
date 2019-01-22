#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Error: Invalid estimation or ground_truth data to calculate RMSE" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
         0,0,0,0,
         0,0,0,0;

  if (x_state.size() != 4)
  {
  	cout << "Error: cannot calculate Jacobian X_state is not 4 sized" << endl;
  	return Hj;
  }
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float px_py_2 = px*px + py*py;
  float sq_px_py_2 = sqrt(px_py_2);
  float v_x_p = vx*py - vy*px;
  float v_x_p_i = - vx*py + vy*px;
  float px_py_3_2 = px_py_2*sq_px_py_2;

  // check division by zero
  if((px==0)&(py==0))
  {
      cout<< "Error: cannot calculate Jacobian. Division by zero"<<endl;
      return Hj;

  }
  // compute the Jacobian matrix
  else
  {
    Hj<< px/sq_px_py_2,      py/sq_px_py_2,          0,             0,
        -py/px_py_2,         px/px_py_2,             0,             0,
         py*v_x_p/px_py_3_2, py*(v_x_p_i)/px_py_3_2, px/sq_px_py_2, py/sq_px_py_2;
  }

  return Hj;
}
