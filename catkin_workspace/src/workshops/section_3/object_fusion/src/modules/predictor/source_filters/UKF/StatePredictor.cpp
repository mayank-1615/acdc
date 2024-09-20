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

#include "StatePredictor.h"
#include "definitions/utility/ika_utilities.h"
#include <Eigen/Dense>
#include <cmath>
    
constexpr int STATE_SIZE = 10; // Assuming the state vector size is 10
constexpr int SIGMA_POINTS = 2 * STATE_SIZE + 1;

StatePredictor::StatePredictor(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(data, name) {}

// Prediction step for each object
void StatePredictor::runSingleSensor() {
    for (auto &globalObject : data_->object_list_fused.objects) {
    	// IkaUtilities::convertMotionModel(globalObject, 4);
        Eigen::VectorXf x_hat_G = IkaUtilities::getEigenStateVec(&globalObject); // estimated global state
        Eigen::MatrixXf P = globalObject.P();

        // Predict the state and covariance Update global object state and covariance using the UKF
        std::tie(x_hat_G, P) = predictUKF(x_hat_G, P, data_->prediction_gap_in_seconds);
        
        // Update global object state and covariance
        // globalObject.stateVector() = x_hat_G; // Directly update the state vector
        // globalObject.P() = P; // Directly update the covariance matrix

        // Update global object time stamp
        globalObject.header.stamp = data_->object_list_measured.header.stamp;
    }

    // Update global object list time stamp
    data_->object_list_fused.header.stamp = data_->object_list_measured.header.stamp;
}

// Define the process noise covariance matrix (assuming it is diagonal for simplicity)
Eigen::MatrixXf StatePredictor::processNoiseCovariance(float dt) {
    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
    // Q.diagonal().array() = 0.1; // Set all diagonal elements to 0.1
    // Position noise: related to velocity and small random disturbances
    float position_noise = 0.5 * 9.81* dt * dt;  // Depends on max accln and time step
    Q(0, 0) = position_noise;  // x position noise
    Q(1, 1) = position_noise;  // y position noise
    Q(2, 2) = position_noise;  // z position noise 

    // Velocity noise: moderate noise due to acceleration dynamics
    float velocity_noise = 9.81 * dt;  // Based on acceleration and dynamics
    Q(3, 3) = velocity_noise;  // v velocity noise

    // Acceleration noise: small noise due to abrupt changes
    float acceleration_noise = 0.1 * dt;
    Q(4, 4) = acceleration_noise;  // a acceleration noise

    // Heading noise: moderate noise due to steering inaccuracies
    float heading_noise = 0.1 * dt;
    Q(5, 5) = heading_noise;  // heading angle noise

    // Yaw rate noise: related to steering dynamics
    float yaw_rate_noise = 1 * dt;
    Q(6, 6) = yaw_rate_noise;  // yaw rate noise

    // Length, width, height: very small noise as these are almost constant
    float dimension_noise = 0.001 * dt;  // Very low noise for static dimensions
    Q(7, 7) = dimension_noise;  // length noise
    Q(8, 8) = dimension_noise;  // width noise
    Q(9, 9) = dimension_noise;  // height noise
    return Q * dt;
}

// UKF Sigma points generation
void StatePredictor::generateSigmaPoints(const Eigen::VectorXf &x, const Eigen::MatrixXf &P, float lambda, Eigen::MatrixXf &Xsig) {
    Eigen::MatrixXf sqrtP = (P + Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE) * 1e-6).llt().matrixL(); // added regularization for numerical stability
    Xsig.col(0) = x;
    for (int i = 0; i < STATE_SIZE; ++i) {
        Xsig.col(i + 1) = x + sqrt(lambda + STATE_SIZE) * sqrtP.col(i);
        Xsig.col(i + 1 + STATE_SIZE) = x - sqrt(lambda + STATE_SIZE) * sqrtP.col(i);
    }
}

// UKF Prediction Step
std::tuple<Eigen::VectorXf, Eigen::MatrixXf> StatePredictor::predictUKF(Eigen::VectorXf &x, Eigen::MatrixXf &P, float dt) {
    const float alpha = 1e-3;
    const float beta = 2.0;
    const float kappa = 0;
    // const float lambda = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
    const float lambda = 3 - STATE_SIZE;

    // Generate sigma points
    Eigen::MatrixXf Xsig = Eigen::MatrixXf(STATE_SIZE, SIGMA_POINTS);
    generateSigmaPoints(x, P, lambda, Xsig);

    // Predict sigma points
    Eigen::MatrixXf Xsig_pred = Eigen::MatrixXf(STATE_SIZE, SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        Xsig_pred.col(i) = stateTransitionFunction(Xsig.col(i), dt);
    }

    // Predicted state mean
    Eigen::VectorXf x_pred = Eigen::VectorXf::Zero(STATE_SIZE);
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
        x_pred += weights_m(i) * Xsig_pred.col(i);
    }

    // Predicted state covariance
    Eigen::MatrixXf P_pred = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        Eigen::VectorXf diff = Xsig_pred.col(i) - x_pred;
        P_pred += weights_c(i) * diff * diff.transpose();
    }

    // Add process noise covariance
    P_pred += processNoiseCovariance(dt);
    
    // Debugging output
    // std::cout << "Predicted state: " << x_pred.transpose() << std::endl;
    // std::cout << "Predicted covariance: " << P_pred << std::endl;

    // Update state and covariance
    // x = x_pred;
    // P = P_pred;
    // Return updated state and covariance
    return std::make_tuple(x_pred, P_pred);
}

Eigen::VectorXf StatePredictor::stateTransitionFunction(const Eigen::VectorXf &x, float dt) {
    Eigen::VectorXf x_new = x;

    float posX = x(0);
    float posY = x(1);
    float heading = x(5);
    float absVel = x(3);
    float yawrate = x(6);
    float absAcc = x(4);

    // Precompute reused expressions
    float dpsi = yawrate * dt;
    float dpsi2 = yawrate * yawrate;
    float v_dpsi = absVel * yawrate;
    float a_dpsi_dt = absAcc * yawrate * dt;

    if (std::abs(yawrate) > 1e-5) {
        // Using the given formulas
        x_new(0) = posX + (1 / dpsi2) * ( -(yawrate * absVel * sin(heading)) - (absAcc * cos(heading))+ (absAcc * cos ((dpsi +  heading))) + (dpsi * absAcc + yawrate * absVel) * (sin ((dpsi + heading))));
        x_new(1) = posY + (1 / dpsi2) * ( (yawrate * absVel * cos(heading)) - (absAcc * sin(heading)) + (absAcc * sin ((dpsi +  heading))) - (dpsi * absAcc + yawrate * absVel) * (cos ((dpsi + heading))));
    } else {
        // Special case when yawrate is very small
        x_new(0) = posX + absVel * dt * cos (heading) + 0.5 * absAcc * dt * dt * cos (heading);
        x_new(1) = posY + absVel * dt * sin (heading) + 0.5 * absAcc * dt * dt * sin (heading);
    }

    x_new(5) = heading + dpsi;
    x_new(3) = absVel + absAcc * dt;
    x_new(6) = yawrate;
    x_new(4) = absAcc;
        
    return x_new;
}


