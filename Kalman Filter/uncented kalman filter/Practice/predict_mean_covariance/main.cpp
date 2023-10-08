#include "Dense"
#include "ukf.h"
#include "ukf.cpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {

  UKF ukf;

  VectorXd x_pred = VectorXd(5);
  MatrixXd P_pred = MatrixXd(5, 5);
  ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

  return 0;
}
