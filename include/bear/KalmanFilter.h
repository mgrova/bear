//---------------------------------------------------------------------------------------------------------------------
//  KF ABSTRACT CLASS (MICO)
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

#ifndef BEAR_KALMAN_FILTER_SIMPLE_H_
#define BEAR_KALMAN_FILTER_SIMPLE_H_

#include <Eigen/Eigen>

namespace bear {
	
	class KalmanFilter{
	public:
		KalmanFilter();

		void setUpKF( const Eigen::Matrix<double, 4,4 > _Q  , 
					  const Eigen::Matrix<double, 4,4 >	_R  , 
					  const Eigen::Matrix<double, 4,1 > _x0 );

		void setUpKF( const Eigen::Matrix<double, 4,1 > _x0 );

		// last filtered estimation
		Eigen::Matrix<double, 4, 1> state();

	public:
		void stepKF(const Eigen::Matrix<double, 4, 1 > & _Zk, const double _incT);

	protected:

		// KF steps.
		void predictionStep(const double _incT);
		void updateStep(const Eigen::Matrix<double, 4, 1 >&_Zk , const double _incT);

	protected:
		Eigen::Matrix<double, 4, 1 > Xfk_, Xak_;    // actual and forecast state at time k
		Eigen::Matrix<double, 4, 4 > K_;			// Kalman gain
		Eigen::Matrix<double, 4, 4 > Pak_, Pfk_;	// actual and forecast prediction matrix at time k (sigma)
		Eigen::Matrix<double, 4, 4 > Q_;			// Observation covariance (sensors)
		Eigen::Matrix<double, 4, 4 > R_;			// Prediction covariance (model)

        Eigen::Matrix<double, 4, 4 > A_;
		Eigen::Matrix<double, 4, 1 > B_;
		Eigen::Matrix<double, 4, 4 > C_;
        
	};
}

#endif	
