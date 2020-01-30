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

namespace bear {
	
	//-----------------------------------------------------------------------------
	KalmanFilter::KalmanFilter(){
		// Model matrix
		A_.setIdentity();
		B_ = (Eigen::Matrix<double,4,1>() << 1, 0, 0, 0).finished();
		C_.setIdentity();

		// Covariance matrix
		double q = 0.1;
		Q_ = (Eigen::Matrix<double,4,4>() << q*q, 0   , 0    , 0   ,
											0   , q*q , 0    , 0   ,
											0   , 0   , q*q  , 0   ,
											0   , 0   , 0    , q*q ).finished();

		double r = 0.01;
		R_ = (Eigen::Matrix<double,4,4>() << r*r, 0   , 0      , 0   ,
											0   , r*r , 0      , 0   ,
											0   , 0   , 2*r*r  , 0   ,
											0   , 0   , 0      , 2*r*r ).finished();

		K_.setZero();
		P_.setIdentity();
	}
	//-----------------------------------------------------------------------------
	void KalmanFilter::setUpKF(const Eigen::Matrix<double, 4, 4 > 	_Q, 
								const Eigen::Matrix<double, 4, 4 >	_R, 
								const Eigen::Matrix<double, 4, 1 > 	_x0){
		Q_ = _Q;
		R_ = _R;
		Xak_ = _x0;
		Xfk_ = _x0;
	}

	//-----------------------------------------------------------------------------
	void KalmanFilter::setUpKF(const Eigen::Matrix<double, 4, 1 > 	_x0){
		Xak_ = _x0;
		Xfk_ = _x0;
	}
		
	//-----------------------------------------------------------------------------
	Eigen::Matrix<double,4,1> KalmanFilter::state() const{
		return Xak_;
	}

	//-----------------------------------------------------------------------------
	void KalmanFilter::stepKF(const Eigen::Matrix<double, 4, 1 > & _Zk, const double _incT){
		predictionStep(_incT);
		updateStep(_Zk , _incT);
	}

	//-----------------------------------------------------------------------------
	void KalmanFilter::predictionStep(const double _incT){
		A_ = (Eigen::Matrix<double,4,4>() << 1 , 0 , _incT , 0    ,
										     0 , 1 , 0     , _incT,
										     0 , 0 , 1     , 0    ,
										     0 , 0 , 0     , 1    ).finished();
		
		Xfk_ = A_ * Xak_ + B_; 	
		P_   = A_ * P_ * A_.transpose() + R_;
	}

	//-----------------------------------------------------------------------------
	void KalmanFilter::updateStep(const Eigen::Matrix<double, 4, 1 >&_Zk , const double _incT){
		C_ = (Eigen::Matrix<double,4,4>()<< 1 , 0 , 0      ,    0   ,
											0 , 1 , 0      ,    0   ,
											1 , 0 , -incT_ ,    0   ,
											0 , 1 , 0      , -incT_ ).finished();


		K_ = P_ * C_.transpose() * ((C_ * P_ * C_.transpose() + R_).inverse());	
		Xak_ = Xfk_ + K_ * (_Zk - C_ * Xfk_);

		Eigen::Matrix<double, 4, 4> I; I.setIdentity();
		P_ = (I - K_ * C_) * P_;
	}
	//-----------------------------------------------------------------------------
	
}
