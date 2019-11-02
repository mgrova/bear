//---------------------------------------------------------------------------------------------------------------------
//  EKF IMPLEMENTATION CLASS
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

#ifndef BEAR_EKF_H_
#define BEAR_EKF_H_


#include "bear/ExtendedKalmanFilter.h"

namespace bear{
	// State vector Xk = {x, y, z, vx, vy, vz}
	// Observation vector Zk = {x, y, z}
	class EKF : public  bear::ExtendedKalmanFilter<double,6,3>{
	public:
        bool init();

	protected:
		void updateJf(const double _incT);
		void updateHZk();
		void updateJh();

    private:
		
		Eigen::Matrix<double,6,6> Q_;  // State covariance
  		Eigen::Matrix<double,3,3> R_;  // Observation covariance
  		Eigen::Matrix<double,6,1> X0_; // Initial state

    };
}

#endif
