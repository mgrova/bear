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

namespace bear{
    
    KF3DPosition::KF3DPosition(){

    }

    KF3DPosition::~KF3DPosition(){

    }

    void KF3DPosition::updateA(const double _incT){
        A_ = (Eigen::Matrix<double, 6, 6 >() << 1.0,_incT, 0.0 ,  0.0 , 0.0, 0.0,
                                                0.0, 1.0,  0.0 ,  0.0 , 0.0, 0.0,
                                                0.0, 0.0,  1.0 , _incT, 0.0, 0.0,
                                                0.0, 0.0,  0.0 ,  1.0 , 0.0, 0.0,
                                                0.0, 0.0,  0.0 ,  0.0 , 1.0,_incT,
                                                0.0, 0.0,  0.0 ,  0.0 , 0.0, 1.0).finished();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void KF3DPosition::initKF3DPosition(Eigen::Vector3f _position){
        Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Identity();
        Q.block<2,2>(0,0) = Q.block<2,2>(0,0) * 0.01;  // Ex , Evx
        Q.block<2,2>(2,2) = Q.block<2,2>(2,2) * 0.01;  // Ey , Evy
        Q.block<2,2>(4,4) = Q.block<2,2>(4,4) * 0.01;  // Ez , Evz

        Eigen::Matrix<double,3,3> R  = Eigen::Matrix<double,3,3>::Identity() * 0.1;
        
        Eigen::Matrix<double,3,6> C  = (Eigen::Matrix<double,3,6>() <<  
                                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,             
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,             
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();

        Eigen::Matrix<double,6,1> X0 = (Eigen::Matrix<double,6,1>() << _position[0], 0.0, _position[1], 0.0, _position[2], 0.0).finished();
        
        std::cout << "[KF3DPosition] Initial position-> x: " << X0[0] << " y: " << X0[2] << " z: " << X0[4] << std::endl;

        this->setupKF(Q, R, X0, C);
        
        return;
    }

}