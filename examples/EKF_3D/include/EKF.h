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


#include <bear/ExtendedKalmanFilter.h>


namespace bear{
	// State vector Xk = {x, y, z, vx, vy, vz}
	// Observation vector Zk = {x, y, z}
	class EKF : public  bear::ExtendedKalmanFilter<double,12,6>{
	public:
        EKF(){};

	protected:
		void updateJf(const double _incT);
		void updateHZk();
		void updateJh();

    private:
		Eigen::Vector3d scaleFactor_=Eigen::Vector3d( 1.0 , 1.0 , 1.0 ); 	// _scaleFactor (must be estimated)
		
		// Values taken from -> https://pdfs.semanticscholar.org/7e13/6039c245d21a070869779286675180ac1fba.pdf
		Eigen::Vector3d C1_ =Eigen::Vector3d( 0.7562 , 0.0514 , 1.138);		// _c1 [m/(s^2),m/(s^2),deg/s]
		Eigen::Vector3d C2_ =Eigen::Vector3d(-0.8255 , 0.0451 , 16.41);		// _c2 [m/(s^2),m/(s^2),deg/s]
		Eigen::Vector3d T_  =Eigen::Vector3d( 0.3042 , 0.7573 , 1.565);		//  _t [s, s, s]
		
		Eigen::Matrix<double,12,12> Q_;  // State covariance
  		Eigen::Matrix<double,6,6> R_;  // Observation covariance
  		Eigen::Matrix<double,12,1> X0_; // Initial state

    };
}

#endif
