// ---------------------------------------------------------------------------------------------------------------------
//   BEAR
// ---------------------------------------------------------------------------------------------------------------------
//   Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com
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

#include "KF3DPosition.h"
#include "chrono"

int main(int _argc , char **_argv){

    bear::KF3DPosition kf;

    bool doneFirstEstimation = false;
    Eigen::Vector3f position;

    std::chrono::time_point<std::chrono::system_clock> prevT;
    for(;;){

        Eigen::Vector3f newPosition;
        
        if(!doneFirstEstimation){
            kf.initKF3DPosition(position);
            doneFirstEstimation = true;
        }else{

            auto t1 = std::chrono::system_clock::now();
            auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
            prevT= t1;

            Eigen::Matrix<double, 3,1> Zk = newPosition.cast<double>();
            kf.stepKF(Zk , static_cast<double>(incT));

            Eigen::Matrix<double, 6,1> Xk = kf.state();

            std::cout << "Estimated position-> x: " << Xk[0] << " y: " << Xk[2] << " z: " << Xk[4] << std::endl;

        }
    }


    return -1;
}