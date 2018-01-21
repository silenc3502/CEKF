#include <iostream>
#include "tools.h"

#define EPSILON		0.0001

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

  /* It comes from lecture 22. */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if(estimations.size() == 0)
  {
    cout << "Input is empty" << endl;
    return rmse;
  }

  if(estimations.size() != ground_truth.size())
  {
    cout << "Invalid estimation or ground truth. Data should have the same size" << endl;
    return rmse;
  }

  for(unsigned int i = 0; i < estimations.size(); i++)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  /* It comes from lecture 18 */
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  MatrixXd Hj(3, 4);

  if(fabs(px) < EPSILON && fabs(py) < EPSILON)
  {
    px = EPSILON;
    py = EPSILON;
  }

  float c1 = pow(px, 2) + pow(py, 2);

  if(fabs(c1) < pow(EPSILON, 2))
    c1 = pow(EPSILON, 2);

  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  Hj << (px / c2), (py / c2), 0, 0,
	-(py / c1), (px / c1), 0, 0,
	py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
