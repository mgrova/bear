//---------------------------------------------------------------------------------------------------------------------
//  EKF ABSTRACT CLASS (MICO)
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


#ifndef BEAR_EXTENDEDKALMANFILTER_H_
#define BEAR_EXTENDEDKALMANFILTER_H_

#include <Eigen/Eigen>	// Eigen linear algebra library

namespace bear {
	// Abstrac class that implements Extended Kalman Filter (EKF) pipeline.
	//
	template<typename Type_, int D1_, int D2_>
	class ExtendedKalmanFilter{
	public:
		ExtendedKalmanFilter();		// 666 TODO: initialize matrixes.

		void setUpEKF(	const Eigen::Matrix<Type_, D1_, D1_ > 	_Q, 
						const Eigen::Matrix<Type_, D2_, D2_ >	_R, 
						const Eigen::Matrix<Type_, D1_, 1 > 	_x0);

		//get last filtered estimation
		Eigen::Matrix<Type_, D1_, 1> state() const;

	public:
		void stepEKF(const Eigen::Matrix<Type_, D2_, 1 > & _Zk, const double _incT);

	protected:
		// Non specific functions of the EKF. 
		virtual void updateJf(const double _incT) = 0;
		virtual void updateHZk() = 0;
		virtual void updateJh() = 0;

		// EKF steps.
		void forecastStep(const double _incT);
		void filterStep(const Eigen::Matrix<Type_, D2_, 1 >&_Zk);

	protected:
		Eigen::Matrix<Type_, D1_, 1 > 	Xfk_, Xak_;
		Eigen::Matrix<Type_, D1_, D2_ > K_;
		Eigen::Matrix<Type_, D1_, D1_ > Jf_;
		Eigen::Matrix<Type_, D2_, D1_ > Jh_;
		Eigen::Matrix<Type_, D1_, D1_ > P_;
		Eigen::Matrix<Type_, D1_, D1_ > Q_;
		Eigen::Matrix<Type_, D2_, D2_ > R_;
		Eigen::Matrix<Type_, D2_, 1 >   HZk_;

	};	//	class ExtendedKalmanFilter
}	//	namespace bear

#include <bear/include/ExtendedKalmanFilter.inl>


#endif	
