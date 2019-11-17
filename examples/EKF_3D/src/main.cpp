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
#include <fstream>
#include <string>
#include "EKF.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <iterator>

int main(){

    std::ifstream gps_file("/home/marrcogrova/inspector_gps.txt"); 
    std::ifstream imu_file("/home/marrcogrova/inspector_imuAcc.txt");

    if(!gps_file.is_open()){
        return 0;
    }
    if(!imu_file.is_open()){
        return 0;
    }

    Eigen::Matrix<double,12,12> Q = Eigen::Matrix<double,12,12>::Identity() * 0.1;  // State covariance
	Eigen::Matrix<double,6,6>   R = Eigen::Matrix<double,6,6>::Identity() * 0.1;    // Observation covariance
	Eigen::Matrix<double,12,1>  X0= (Eigen::Matrix<double,12,1>() <<  // Initial state
			    	             	0.0,   0.0,   0.0,             // x,y,z
			    	             	1.0,   1.0,   1.0,             // vx,vy,vz
			    	             	0.0,   0.0,   0.0,             // ax,ay,az
			    	               -0.14, 0.051, 6.935).finished(); // bx,by,bz

    // Kalman filter
    bear::EKF ekf;
    ekf.setUpEKF(Q , R , X0);
    std::chrono::time_point<std::chrono::system_clock> prevT;
    prevT = std::chrono::system_clock::now();

    while(!gps_file.eof() || !imu_file.eof()){
        std::vector<double> GPSPositionObs     = {0.0 , 0.0 , 0.0};
        std::vector<double> ImuAccelerationObs = {0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0};

        std::string DataGPS;
        getline(gps_file,DataGPS);
        std::stringstream ss1(DataGPS);
        ss1 >> GPSPositionObs[0] >> GPSPositionObs[1] >> GPSPositionObs[2];

        std::string DataImu;
        getline(imu_file,DataImu);
        std::stringstream ss2(DataImu);
        ss2 >> ImuAccelerationObs[0] >> ImuAccelerationObs[1] >> ImuAccelerationObs[2] >> ImuAccelerationObs[3] >>
               ImuAccelerationObs[4] >> ImuAccelerationObs[5] >> ImuAccelerationObs[6] >>
               ImuAccelerationObs[7] >> ImuAccelerationObs[8] >> ImuAccelerationObs[9];

        Eigen::Matrix<double, 6,1> Zk = (Eigen::Matrix<double,6,1>() <<
                                    GPSPositionObs[0] , GPSPositionObs[1] , GPSPositionObs[2],
			    	             ImuAccelerationObs[7] , ImuAccelerationObs[8] , ImuAccelerationObs[9]).finished();

        std::cout << Zk << std::endl;

        auto t1 = std::chrono::system_clock::now();
        auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
        ekf.stepEKF(Zk , incT); // face time
        prevT= t1;
        Eigen::Matrix<double, 12,1> Xk = ekf.state();

        std::cout << "Position predicted: " << Xk(0) << " " << Xk(1) << " " << Xk(2) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}