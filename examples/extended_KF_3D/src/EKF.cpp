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

#include "EKF.h"

namespace bear{

	double sign2(double _var) {
		return _var < 0? -1:1;
	}

    void EKF::updateJf(const double _incT){
		Jf_ = Eigen::MatrixXd::Identity(Jf_.rows(), Jf_.cols());
		Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
		Jf_.block<3, 3>(0, 3) =I*_incT;
		Jf_.block<3, 3>(0, 6) =I*_incT*_incT/2;
		Jf_.block<3, 3>(3, 6) =I*_incT;
		
		Eigen::Vector3d auxAcc({sign2(Xak_(6,0)*scaleFactor_[0]), sign2(Xak_(6,0)*scaleFactor_[1]), sign2(Xak_(6,0)*scaleFactor_[2])});
		Eigen::Matrix3d auxMatAcc = (auxAcc*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
		Jf_.block<3, 3>(6, 6) += auxMatAcc;

		Jf_.block<3, 3>(6, 9) =-I;
		
		Eigen::Matrix3d diagT = (T_*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
		Eigen::Matrix3d diagTpluxTime = (diagT + I*_incT);
		Jf_(9, 9) = diagT(0,0)/diagTpluxTime(0,0);
		Jf_(10, 10) = diagT(1,1)/diagTpluxTime(1,1);
		Jf_(11, 11) = diagT(2,2)/diagTpluxTime(2,2);
    }

	void EKF::updateHZk(){
        HZk_.block<3,1>(0,0) = Xfk_.block<3,1>(0,0);
		HZk_.block<3,1>(3,0) = Xfk_.block<3,1>(6,0);    
    }

	void EKF::updateJh(){
		Jh_(0,0) = 1;
		Jh_(1,1) = 1;
		Jh_(2,2) = 1;
		Jh_(3,6) = 1;
		Jh_(4,7) = 1;
		Jh_(5,8) = 1;
    }
}