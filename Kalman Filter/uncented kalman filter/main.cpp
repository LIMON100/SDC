#include <iostream>
#include "Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  MatrixXd Xsig = MatrixXd(7, 15);
  //ukf.GenerateSigmaPoints(&Xsig);
  ukf.AugmentedSigmaPoints(&Xsig);

  // print result
  std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  return 0;
}
