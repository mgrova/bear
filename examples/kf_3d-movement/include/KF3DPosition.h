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

#ifndef KALMAN_FILTER_3D_POSITION_BEAR_H_
#define KALMAN_FILTER_3D_POSITION_BEAR_H_

#include <bear/KalmanFilter.h>

namespace bear{

    // State vector Xk = {x, vx, y, vy, z, vz}
	// Observation vector Zk = {x, y, z}
    class KF3DPosition : public bear::KalmanFilter<double, 6, 3>{
    public:
        KF3DPosition();
        virtual ~KF3DPosition();
        
        void initKF3DPosition(Eigen::Vector3f _position);

    protected:
		void updateA(const double _incT);

    };
    #endif

}