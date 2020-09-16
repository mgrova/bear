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

#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <iostream>
#include <Eigen/Dense>

namespace bear{
    struct BoundingBox{
        unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
        float prob;                    // confidence - probability that the object was found correctly
        unsigned int obj_id;           // class of object - from range [0, classes-1]
        Eigen::Vector3f obj_center3D;  // 3D point of center of the object using stereo matching

        void print(){
            std::cout << "[BoundingBox] BBox of class: " << obj_id << " with conf: " << prob 
                      << " and limits: x = " << x << " | y = " << y
                      << " w = " << w << " | h = " << h << std::endl;
            return;
        }
    };
}

#endif