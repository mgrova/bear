//---------------------------------------------------------------------------------------------------------------------
//  BEAR
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include "EKF.h"
#include <opencv2/opencv.hpp>

int main(){
    const double NOISE_LEVEL = 0.1;

    // Observation covariance matrix
    Eigen::Matrix<double,3,3> R;
    R = Eigen::Matrix<double,3,3>::Identity();
    R.block<3,3>(0,0) *= 0.01*NOISE_LEVEL;

    // Prediction covariance matrix
    Eigen::Matrix<double,6,6> Q;
    Q = Eigen::Matrix<double,6,6>::Identity();
    Q.block<3,3>(0,0) *= 0.001;
    Q.block<3,3>(3,3) *= 0.02;

    // Initial state
    Eigen::Matrix<double,6,1> X0;
    X0.block<1,3>(0,0) = Eigen::Vector3d(1.0 , 0.0 , 0.0); // x,y,z
    X0.block<1,3>(0,3) = Eigen::Vector3d(0.0 , 0.0 , 0.0); // vx,vy,vz

    // Kalman filter
    bear::EKF ekf;
    ekf.setUpEKF(Q , R , X0);

    float fakeTimer = 0;
    while(true){
        Eigen::Matrix<double, 3,1> Zk;    // New observation
        
        ekf.stepEKF(Zk , double(0.03)); // face time

        Eigen::Matrix<double, 6,1> Xk = ekf.state();

        fakeTimer += .03;
    }

    return 0;
}