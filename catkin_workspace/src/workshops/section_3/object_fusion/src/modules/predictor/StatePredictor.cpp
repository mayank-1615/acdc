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
    

StatePredictor::StatePredictor(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(data, name) {}

// Prediction step for each object
void StatePredictor::runSingleSensor() {
    // Prediction step for each object
    for (auto &globalObject : data_->object_list_fused.objects) {
    	// IkaUtilities::convertMotionModel(globalObject, 4);
        auto x_hat_G = IkaUtilities::getEigenStateVec(&globalObject); // estimated global state

        // Calculate the predicted state using the state transition function
        x_hat_G = stateTransitionFunction(x_hat_G, data_->prediction_gap_in_seconds);

        // Calculate the Jacobian matrix
        Eigen::MatrixXf F = jacobianFunction(x_hat_G, data_->prediction_gap_in_seconds);

        // Process noise covariance matrix (as per ACDC course keeping it as identity)
        Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(10, 10);
    	// Q.diagonal().array() = 0.1; // Set all diagonal elements to 0.1
    	// Position noise: related to velocity and small random disturbances
    	float position_noise = 0.5 * 9.81* data_->prediction_gap_in_seconds * data_->prediction_gap_in_seconds;  // Depends on max accln and time step
    	Q(0, 0) = position_noise;  // x position noise
    	Q(1, 1) = position_noise;  // y position noise
    	Q(2, 2) = position_noise;  // z position noise 

    	// Velocity noise: moderate noise due to acceleration dynamics
    	float velocity_noise = 9.81 * data_->prediction_gap_in_seconds;  // Based on acceleration and dynamics
    	Q(3, 3) = velocity_noise;  // v velocity noise

    	// Acceleration noise: small noise due to abrupt changes
    	float acceleration_noise = 0.1 * data_->prediction_gap_in_seconds;
    	Q(4, 4) = acceleration_noise;  // a acceleration noise

    	// Heading noise: moderate noise due to steering inaccuracies
    	float heading_noise = 0.1 * data_->prediction_gap_in_seconds;
    	Q(5, 5) = heading_noise;  // heading angle noise

    	// Yaw rate noise: related to steering dynamics
    	float yaw_rate_noise = 1 * data_->prediction_gap_in_seconds;
    	Q(6, 6) = yaw_rate_noise;  // yaw rate noise

    	// Length, width, height: very small noise as these are almost constant
    	float dimension_noise = 0.001 * data_->prediction_gap_in_seconds;  // Very low noise for static dimensions
    	Q(7, 7) = dimension_noise;  // length noise
    	Q(8, 8) = dimension_noise;  // width noise
    	Q(9, 9) = dimension_noise;  // height noise
  
        // Predict the covariance matrix
        globalObject.P() = F * globalObject.P() * F.transpose() + Q * data_->prediction_gap_in_seconds;

        // Update global object time stamp
        globalObject.header.stamp = data_->object_list_measured.header.stamp;
    }

    // Update global object list time stamp
    data_->object_list_fused.header.stamp = data_->object_list_measured.header.stamp;
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
        x_new(0) = posX + (1 / dpsi2) * ( -(yawrate * absVel * sin(heading)) - (absAcc * cos(heading)) + (absAcc * cos (dpsi +  heading)) + (dpsi * absAcc + yawrate * absVel) * (sin (dpsi + heading)));
        x_new(1) = posY + (1 / dpsi2) * ( (yawrate * absVel * cos(heading)) - (absAcc * sin(heading)) + (absAcc * sin (dpsi +  heading)) - (dpsi * absAcc + yawrate * absVel) * (cos (dpsi + heading)));
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


Eigen::MatrixXf StatePredictor::jacobianFunction(const Eigen::VectorXf &x, float dt) {
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(10, 10);

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
    float sin_psi_dpsi = sin(heading + dpsi);
    float cos_psi_dpsi = cos(heading + dpsi);

    if (std::abs(yawrate) > 1e-5) {
        // Derivatives for x position
        F(0, 0) = 1.0;
        F(0, 5) = (1 / dpsi2) * (-(yawrate * absVel * cos (heading)) +  (absAcc * sin (heading)) - (absAcc * sin (heading + dpsi)) + (dt * yawrate * absAcc +  yawrate * absVel) * cos (heading + dpsi));
        F(0, 3) = (1 / dpsi2) * (- (yawrate * sin(heading)) + yawrate * sin (heading + dpsi));
        F(0, 6) = (1 / dpsi2) * (-(dt * absAcc * sin (heading + dpsi)) +  (dt * (dt * yawrate * absAcc + yawrate * absVel) * cos (heading + dpsi)) - (absVel * sin (heading)) + (dt * absAcc + absVel) * sin (heading + dpsi)) - 2 * (1 / (dpsi2 * yawrate)) * (-(yawrate * absVel * sin (heading)) -  (absAcc * cos (heading)) + (absAcc * cos (heading + dpsi)) + (dt * yawrate * absAcc + yawrate * absVel) * sin (heading + dpsi));
        F(0, 4) = (1 / dpsi2) * ((yawrate * dt * sin (heading + dpsi)) - (cos (heading)) + cos (heading + dpsi));

        // Derivatives for y position
        F(1, 1) = 1.0;
        F(1, 5) = (1 / dpsi2) * (-(yawrate * absVel * sin (heading)) -  (absAcc * cos (heading)) + (absAcc * cos (heading + dpsi)) + (dt * yawrate * absAcc +  yawrate * absVel) * sin (heading + dpsi));
        F(1, 3) = (1 / dpsi2) * ((yawrate * cos(heading)) - yawrate * cos (heading + dpsi));
        F(1, 6) = (1 / dpsi2) * ((dt * absAcc * cos (heading + dpsi)) +  (dt * (dt * yawrate * absAcc + yawrate * absVel) * sin (heading + dpsi)) + (absVel * cos (heading)) + (dt * absAcc + absVel) * cos (heading + dpsi)) - 2 * (1 / (dpsi2 * yawrate)) * ((yawrate * absVel * cos (heading)) -  (absAcc * sin (heading)) + (absAcc * sin (heading + dpsi)) + (dt * yawrate * absAcc + yawrate * absVel) * cos (heading + dpsi));
        F(1, 4) = (1 / dpsi2) * (- (yawrate * dt * cos (heading + dpsi)) - (sin (heading)) + sin (heading + dpsi));

        // Derivative for heading
        F(5, 6) = dt;

        // Derivative for velocity
        F(3, 4) = dt;
    } else {
        // Special case when yawrate is very small
        F(0, 3) = dt * cos(heading);
        F(0, 5) = -absVel * dt * sin(heading);
        F(0, 4) = 0.5 * dt * dt * cos(heading);
        
        F(1, 3) = dt * sin(heading);
        F(1, 5) = absVel * dt * cos(heading);
        F(1, 4) = 0.5 * dt * dt * sin(heading);

        // Derivative for heading
        F(5, 6) = dt;

        // Derivative for velocity
        F(3, 4) = dt;
    }

    return F;
}


