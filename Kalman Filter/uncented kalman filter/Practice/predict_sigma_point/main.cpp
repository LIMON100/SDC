#include "Dense"
#include "ukf.h"
#include "ukf.cpp"

using Eigen::MatrixXd;

int main() {

  UKF ukf;

  MatrixXd Xsig_pred = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_pred);

  return 0;
}
