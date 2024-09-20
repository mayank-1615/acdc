// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "StateFuser.h"
#include "data/Params.h"
#include <definitions/utility/ika_utilities.h>

constexpr int STATE_SIZE = 10; // Assuming the state vector size is 10
constexpr int SIGMA_POINTS = 2 * STATE_SIZE + 1;

StateFuser::StateFuser(std::shared_ptr<Data> data, std::string name)
: AbstractFusionModule(data, name) {}

Eigen::MatrixXf StateFuser::constructMeasurementMatrix() {

  // find valid object in measured object list
  Eigen::VectorXf variance;
  bool foundValidObject = false;
  for (auto &measuredObject: data_->object_list_measured.objects) {
    if (measuredObject.bObjectValid) {
      // IkaUtilities::convertMotionModel(measuredObject, 4);
      variance = IkaUtilities::getEigenVarianceVec(&measuredObject);
      foundValidObject = true;
    }
  }

  if (!foundValidObject) {
    return Eigen::MatrixXf::Zero(0,variance.size());
  }

  int row_size = 0;
  for (int i = 0; i < variance.size(); ++i) {
    if (variance[i] >= 0) {
      ++row_size;
    }
  }
  if (row_size == 0) return Eigen::MatrixXf::Zero(0, variance.size());

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(long(row_size), variance.size());

  int row = 0;
  for (int col = 0; col < variance.size() && row < row_size; ++col) {
    if (variance[col] >= 0) {
      C(row, col) = 1;
      row++;
    }
  }
  return C;
}

// UKF Sigma points generation
void StateFuser::generateSigmaPoints(const Eigen::VectorXf &x, const Eigen::MatrixXf &P, float lambda, Eigen::MatrixXf &Xsig) {
    Eigen::MatrixXf sqrtP = (P + Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE) * 1e-6).llt().matrixL();
    Xsig.col(0) = x;
    for (int i = 0; i < STATE_SIZE; ++i) {
        Xsig.col(i + 1) = x + sqrt(lambda + STATE_SIZE) * sqrtP.col(i);
        Xsig.col(i + 1 + STATE_SIZE) = x - sqrt(lambda + STATE_SIZE) * sqrtP.col(i);
    }
}

void StateFuser::runSingleSensor() {
  // Construct measurement matrix C
  Eigen::MatrixXf C = constructMeasurementMatrix();
  if (C.rows() == 0 || C.cols() == 0) {
    // No valid measurement matrix, exit the function
    return;
  }

  int count = -1;
  for (auto &globalObject : data_->object_list_fused.objects) {
    count++;

    auto x_hat_G = IkaUtilities::getEigenStateVec(&globalObject); // predicted global state
    // IkaUtilities::convertMotionModel(globalObject, 4);

    int measurementIndex = data_->associated_measured[count];
    if (measurementIndex < 0) {
      continue; // no associated measurement
    }

    definitions::IkaObject& measuredObject = data_->object_list_measured.objects[measurementIndex];
    auto P_S_diag = IkaUtilities::getEigenVarianceVec(&measuredObject); // predicted measured state variance
    	
    Eigen::MatrixXf R_diag = C * P_S_diag;
    Eigen::MatrixXf R = R_diag.asDiagonal();

    // UKF parameters
    const float alpha = 1e-3;
    const float beta = 2.0;
    const float kappa = 0;
    // const float lambda = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
    const float lambda = 3 - STATE_SIZE;

    // Generate sigma points for the predicted state
    Eigen::MatrixXf Xsig_pred = Eigen::MatrixXf(STATE_SIZE, SIGMA_POINTS);
    generateSigmaPoints(x_hat_G, globalObject.P(), lambda, Xsig_pred);

    // Predict measurement sigma points
    Eigen::MatrixXf Zsig = Eigen::MatrixXf(C.rows(), SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
      Zsig.col(i) = C * Xsig_pred.col(i);
    }

    // Calculate mean predicted measurement
    Eigen::VectorXf z_pred = Eigen::VectorXf::Zero(C.rows());
    Eigen::VectorXf weights = Eigen::VectorXf(SIGMA_POINTS);
    Eigen::VectorXf weights_m = Eigen::VectorXf(SIGMA_POINTS);
    Eigen::VectorXf weights_c = Eigen::VectorXf(SIGMA_POINTS);
    weights(0) = lambda / (lambda + STATE_SIZE);
    weights_m(0) = lambda / (lambda + STATE_SIZE);
    weights_c(0) = lambda / (lambda + STATE_SIZE) + (1 - alpha*alpha + beta);
    for (int i = 1; i < SIGMA_POINTS; ++i) {
        weights(i) = 0.5 / (STATE_SIZE + lambda);
        weights_m(i) = 0.5 / (STATE_SIZE + lambda);
        weights_c(i) = 0.5 / (STATE_SIZE + lambda);
    }
    
    for (int i = 0; i < SIGMA_POINTS; ++i) {
      z_pred += weights_m(i) * Zsig.col(i);
    }

    // Calculate measurement covariance matrix S
    Eigen::MatrixXf S = Eigen::MatrixXf::Zero(C.rows(), C.rows());
    for (int i = 0; i < SIGMA_POINTS; ++i) {
      Eigen::VectorXf diff = Zsig.col(i) - z_pred;
      S += weights_c(i) * diff * diff.transpose();
    }
    S += R;

    // Calculate cross-correlation matrix
    Eigen::MatrixXf Tc = Eigen::MatrixXf::Zero(STATE_SIZE, C.rows());
    for (int i = 0; i < SIGMA_POINTS; ++i) {
      Eigen::VectorXf x_diff = Xsig_pred.col(i) - x_hat_G;
      Eigen::VectorXf z_diff = Zsig.col(i) - z_pred;
      Tc += weights_c(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain
    Eigen::MatrixXf K = Tc * S.inverse();
    // Print the Kalman gain matrix
    std::cout << "Kalman gain matrix K:\n" << K << std::endl;

    // Actual measurement (z)
    Eigen::VectorXf x_hat_S = IkaUtilities::getEigenStateVec(&measuredObject);
    Eigen::VectorXf z = C * x_hat_S;

    // Update state estimate
    x_hat_G += K * (z - z_pred);

    // Update global matrix P
    globalObject.P() -= K * S * K.transpose();
  }
}
