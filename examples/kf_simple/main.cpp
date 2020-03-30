// ---------------------------------------------------------------------------------------------------------------------
//   BEAR
// ---------------------------------------------------------------------------------------------------------------------
//   Copyright 2019 ViGUS University of Seville
// ---------------------------------------------------------------------------------------------------------------------
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//   and associated documentation files (the "Software"), to deal in the Software without restriction,
//   including without limitation the rights to use, copy, modify, merge, publish, distribute,
//   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
// 
//   The above copyright notice and this permission notice shall be included in all copies or substantial
//   portions of the Software.
// 
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//   BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//   OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// ---------------------------------------------------------------------------------------------------------------------

#include<bear/KalmanFilter.h>
#include <iostream>
#include <thread>

int main(int _argc , char **_argv){

    bear::KalmanFilter<double, 4, 4> kf;

    // Eigen::Matrix<double, 4,1 > x0 = (Eigen::Matrix<double, 4,1 >() << 0.0 , 0.0, 0.0, 0.0).finished();

    // kf = new bear::KalmanFilter;
    // kf->setUpKF(x0);

    // std::chrono::time_point<std::chrono::system_clock> prevT;
    // prevT = std::chrono::system_clock::now();

    // std::vector<double> pos = {0.0 , 0.0};
    // std::vector<double> vel = {1.0 , 0.0};
    
    // while(true){
    //     // pos[0] = pos[0] + vel[0]*0.1;
    //     // pos[1] = pos[1] + vel[1]*0.1;

    //     Eigen::Matrix<double, 4,1> Zk = (Eigen::Matrix<double,4,1>() <<
    //                                 pos[0] , pos[1] , vel[0] , vel[1]).finished();

    //     auto t1 = std::chrono::system_clock::now();
    //     auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
    //     kf->stepKF(Zk , static_cast<double>(incT));
    //     prevT= t1;
    //     Eigen::Matrix<double, 4,1> Xk = kf->state();

    //     std::cout << "Position predicted: " << Xk(0) << " " << Xk(1)  << std::endl;
    //     std::cout << "Velocidad predicted: " << Xk(2) << " " << Xk(3)  << std::endl;

    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

    return 0;
}