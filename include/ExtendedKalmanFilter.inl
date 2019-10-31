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

#include <iostream>

namespace mico {
	
	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	ExtendedKalmanFilter<Type_, D1_, D2_>::ExtendedKalmanFilter(){

	}
		
	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	void ExtendedKalmanFilter<Type_, D1_, D2_>::setUpEKF(	const Eigen::Matrix<Type_, D1_, D1_ > 	_Q, 
															const Eigen::Matrix<Type_, D2_, D2_ >	_R, 
															const Eigen::Matrix<Type_, D1_, 1 > 	_x0){
		Q_ = _Q;
		R_ = _R;
		Xak_ = _x0;
		Xfk_ = _x0;
		K_.setZero();
		Jf_.setIdentity();
		P_.setIdentity();
		HZk_.setZero();
		Jh_.setZero();
	}
		
	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	Eigen::Matrix<Type_, D1_, 1> ExtendedKalmanFilter<Type_, D1_, D2_>::state() const{
		return Xak_;
	}

	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	void ExtendedKalmanFilter<Type_, D1_, D2_>::stepEKF(const Eigen::Matrix<Type_, D2_, 1 > & _Zk, const double _incT){
		forecastStep(_incT);
		filterStep(_Zk);
	}

	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	void ExtendedKalmanFilter<Type_, D1_, D2_>::forecastStep(const double _incT){
		updateJf(_incT);
		
		Xfk_ = Jf_ * Xak_;
		P_   = Jf_ * P_ * Jf_.transpose() + Q_;
	}

	//-----------------------------------------------------------------------------
	template<typename Type_, int D1_, int D2_>
	void ExtendedKalmanFilter<Type_, D1_, D2_>::filterStep(const Eigen::Matrix<Type_, D2_, 1 >&_Zk){
		updateHZk();
		updateJh();
		K_ = P_ * Jh_.transpose() * ((Jh_ * P_ * Jh_.transpose() + R_).inverse());
		Xak_ = Xfk_ + K_ * (_Zk - HZk_);
		Eigen::Matrix<Type_, D1_, D1_> I; I.setIdentity();
		P_ = (I - K_ * Jh_) * P_;
	}
	//-----------------------------------------------------------------------------
	//-----------------------------------------------------------------------------
}
