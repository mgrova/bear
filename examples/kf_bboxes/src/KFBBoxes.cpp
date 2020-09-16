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

#include "KFBBoxes.h"

namespace bear{
    
    KFBBoxes::KFBBoxes(){

    }

    KFBBoxes::~KFBBoxes(){

    }
    
    //---------------------------------------------------------------------------------------------------------------------
    void KFBBoxes::updateA(const double _incT){
        A_ = (Eigen::Matrix<double, 6, 6 >() << 1.0, 0.0, _incT,  0.0 , 0.0, 0.0,
                                                0.0, 1.0,  0.0 , _incT, 0.0, 0.0,
                                                0.0, 0.0,  1.0 ,  0.0 , 0.0, 0.0,
                                                0.0, 0.0,  0.0 ,  1.0 , 0.0, 0.0,
                                                0.0, 0.0,  0.0 ,  0.0 , 1.0, 0.0,
                                                0.0, 0.0,  0.0 ,  0.0 , 0.0, 1.0).finished();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void KFBBoxes::initKFBboxes(BoundingBox _detection){
        Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Identity();
        Q.block<2,2>(0,0) = Q.block<2,2>(0,0) * 0.01;   // Ex , Ey
        Q.block<2,2>(2,2) = Q.block<2,2>(2,2) * 0.01;   // Evx, Evy
        Q.block<2,2>(4,4) = Q.block<2,2>(4,4) * 0.005;  // Ew , Eh

        Eigen::Matrix<double,4,4> R  = Eigen::Matrix<double,4,4>::Identity() * 0.1;
        
        Eigen::Matrix<double,4,6> C  = (Eigen::Matrix<double,4,6>() <<  
                                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,             
                                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,             
                                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,             
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();

        Eigen::Matrix<double,6,1> X0 = (Eigen::Matrix<double,6,1>() << _detection.x, _detection.y, 0.0, 0.0, _detection.w, _detection.h).finished();

        std::cout << "[KFBBoxes] Initial bounding box -> x: " << _detection.x << " y: " << _detection.y 
                                                    << " w: " << _detection.w << " h: " << _detection.h << std::endl;

        this->setupKF(Q, R, X0, C);
        
        return;
    }

}