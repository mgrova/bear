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
#include <iostream>

namespace bear {
	
	template<typename _type, int _D1, int _D2>
	class KalmanFilter{
	public:
		KalmanFilter();

		void setupKF( const Eigen::Matrix<_type, _D1, _D1 > _Q , 
					  const Eigen::Matrix<_type, _D2, _D2 >	_R , 
					  const Eigen::Matrix<_type, _D1,  1  > _x0,
					  const Eigen::Matrix<_type, _D2, _D1 > _C  );
		

		// last filtered estimation
		Eigen::Matrix<_type, _D1, 1> state() const;

	public:
		
		void stepKF(const Eigen::Matrix<_type, _D2, 1 > & _Zk, const _type _incT);
		void stepKF(const _type _incT);

	protected:
	 	// Non specific functions of the EKF. 
		virtual void updateA(const double _incT) = 0;
		
		// KF steps 
		void predictionStep(const _type _incT);
		void updateStep(const Eigen::Matrix<_type, _D2, 1 >&_Zk);

	protected:
		Eigen::Matrix<_type, _D1,  1  > Xfk_, Xak_; // actual and forecast state at time k
		Eigen::Matrix<_type, _D1, _D2 > K_;			// Kalman gain
		Eigen::Matrix<_type, _D1, _D1 > Pak_, Pfk_;	// actual and forecast prediction matrix at time k (sigma)
		Eigen::Matrix<_type, _D1, _D1 > Q_;			// Prediction covariance (model)
		Eigen::Matrix<_type, _D2, _D2 > R_;			// Observation covariance (sensors)

        Eigen::Matrix<_type, _D1, _D1 > A_;		// Transition state matrix
		Eigen::Matrix<_type, _D1,  1  > B_;
		Eigen::Matrix<_type, _D2, _D1 > C_;		// Measure matrix
        
	};
}

#include <bear/KalmanFilter.inl>

#endif	
