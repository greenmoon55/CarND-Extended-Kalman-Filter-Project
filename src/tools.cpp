#include <iostream>
#include "tools.h"
#define EPS 0.0001
#define EPS2 0.0000001

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0)
      return rmse;
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size())
      return rmse;
  // ... your code here

  //accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  // ... your code here
  rmse = rmse / estimations.size();

  //calculate the squared root
  // ... your code here
  rmse = rmse.array().sqrt();
    
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj = MatrixXd::Zero(3,4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(px) < EPS || fabs(py) < EPS) {
      px = EPS;
      py = EPS;
  } 
  // compute the Jacobian matrix
  else {
    if (c1 < EPS) {
      c1 = EPS2;
      // Hj <<    0,    0, 0, 0,
      //   1e+9, 1e+9, 0, 0,
      //     0,    0, 0, 0;
      // return Hj;
    }
    Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
    // Hj(0, 0) = px / sqrt(sum);
    // Hj(0, 1) = py / sqrt(sum);
    // Hj(1, 0) = -py / sum;
    // Hj(1, 1) = px / sum;
    // Hj(2, 0) = py* (vx*py - vy*px) / (sum*sqrt(sum));
    // Hj(2, 1) = px* (vy*px - vx*py) / (sum*sqrt(sum));
    // Hj(2, 2) = Hj(0, 0);
    // Hj(2, 3) = Hj(0, 1);
    cout << px << " " << py << " " << c1 << endl;
  }
  return Hj;
}
