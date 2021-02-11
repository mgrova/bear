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

#ifdef HAS_DARKNET
#include <yolo_v2_class.hpp>
#endif

#include <iostream>
#include <thread>

#ifdef HAS_DARKNET
Detector* detector_;
#endif


#ifdef HAS_DARKNET
bool detectMobile(cv::Mat _image, bear::BoundingBox &_detection){
    std::vector<bbox_t> detections = detector_->detect(_image , 0.2);
    
    bool success = false;
    for(auto det : detections){
        if (det.obj_id == 68 - 1){ // 666 mobile phone id
            _detection.x = det.x; _detection.y = det.y;
            _detection.h = det.h; _detection.w = det.w; 
            _detection.prob = det.prob; 
            _detection.obj_id = det.obj_id;
            
            success = true;
        }
    }

    return success;
}
#endif

int main(int _argc , char **_argv){

#ifdef HAS_DARKNET
    std::string yoloCfg, yoloWeights;
    if(_argc == 3){
        yoloCfg     = _argv[1];
        yoloWeights = _argv[2];
    }else{
        std::cout << "Error reading cfg and weigths.\n example execution: ./example_kf-bboxes <yolo_cfg> <yolo_weigths> \n";
        return -1;

    }

    detector_ = new Detector(yoloCfg , yoloWeights);

    cv::namedWindow("Detections image",cv::WINDOW_AUTOSIZE);
    cvStartWindowThread();

    cv::VideoCapture cap;
    if(!cap.open(0)){
        std::cout << "Error opening video \n";
        return -1;
    }

    bear::KFBBoxes kf;
    std::chrono::time_point<std::chrono::system_clock> prevT;

    bool doneFirstDetection = false;
    for(;;){
        cv::Mat frame;

	    cap >> frame;
	    if (frame.empty())
            break;
        
        bear::BoundingBox det;
        if (detectMobile(frame, det)){
            det.print();
            
            // Print detected bounding box
            cv::Rect rec(det.x , det.y , det.w , det.h);
            cv::rectangle(frame , rec , cv::Scalar(0, 255, 0) , 2);

            // Update/init KF and print extimated bounding box
            if(!doneFirstDetection){
                kf.initKFBboxes(det);
                doneFirstDetection = true;
            }else{
                auto t1 = std::chrono::system_clock::now();
                auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
                prevT = t1;

                Eigen::Matrix<double, 4,1> Zk = (Eigen::Matrix<double,4,1>() << det.x , det.y,
                                                                                det.w , det.h).finished();
                kf.stepKF(Zk , static_cast<double>(incT));

                Eigen::Matrix<double, 6,1> Xk = kf.state();
                std::cout << "Time increment: " << incT << " s | Estimated bounding box-> x: " << Xk[0] << " y: " << Xk[1] << " w: " << Xk[4] << " h: " << Xk[5] << std::endl;
                cv::Rect recKalman(Xk[0], Xk[1], Xk[4], Xk[5]);
                cv::rectangle(frame , recKalman , cv::Scalar(255, 0, 0) , 2);
            }
        }
        
        cv::imshow("Detections image", frame);
    }
    

    cap.release();
    cv::destroyWindow("Detections image");
                
    return -1;
    #else
    std::cerr << "BEAR compiled without darknet" << std::endl;
    #endif
}