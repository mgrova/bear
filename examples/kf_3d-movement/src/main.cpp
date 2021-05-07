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
#ifdef BEAR_HAS_PYTHON
#   define _USE_MATH_DEFINES
#   include <cmath>
#   include "matplotlibcpp.h"
#endif

#include "KF3DPosition.h"

#include <chrono>
#include <fstream>
#include <memory>
#include <thread>

#ifdef BEAR_HAS_PYTHON
void updateVisualization(std::vector<double>& real_x,  std::vector<double>& real_y,  std::vector<double>& real_z, 
                         std::vector<double>& estim_x, std::vector<double>& estim_y, std::vector<double>& estim_z){
    matplotlibcpp::clf();

    matplotlibcpp::suptitle("Obtained vs. Filtered");
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::named_plot("Real", real_x);
    matplotlibcpp::named_plot("Estimated", estim_x);

    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::named_plot("Real", real_y);
    matplotlibcpp::named_plot("Estimated", estim_y);

    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::named_plot("Real", real_z);
    matplotlibcpp::named_plot("Estimated", estim_z);

    matplotlibcpp::pause(0.001);
}
#endif

// Expected format : [x, y, z]
std::vector<Eigen::Vector3f> convertFileToVector(std::string _input){
    std::vector<Eigen::Vector3f> positions;
    std::ifstream infile(_input);
    if (infile.is_open()) {
        float x, y, z;
        char c;
        while (infile >> x >> c >> y >> c >> z && c == ',')
            positions.emplace_back(Eigen::Vector3f(x, y, z));
    }
    infile.close();

    return positions;
}

int main(int _argc , char **_argv){
    if(_argc != 2){
        std::cout << "Error getting path. Usage example: ./example_kf-movement <path_to_csv_file>";
        return 0;
    }

    const std::string inputPath = std::string(_argv[1]);
    std::vector<Eigen::Vector3f> lidarPositions = convertFileToVector(inputPath);
    if(lidarPositions.empty()){
        std::cout << "Error reading data. Exiting..\n";
        return 0;
    }

    // Output format : [x, y, z]
    size_t lastIndex = inputPath.find_last_of("."); 
    std::string rawName = inputPath.substr(0, lastIndex); 
    std::ofstream ouputFile;
    ouputFile.open(rawName + "_filtered.txt");
    
    auto kf = std::make_unique<bear::KF3DPosition>();
    std::chrono::time_point<std::chrono::system_clock> prevT;

    std::vector<double> real_x,  real_y,  real_z;
    std::vector<double> estim_x, estim_y, estim_z;
    bool doneFirstEstimation = false;
    for(const auto& newPosition : lidarPositions){

        if(!doneFirstEstimation){
            kf->initKF3DPosition(newPosition);
            doneFirstEstimation = true;
            prevT = std::chrono::system_clock::now();
        }else{
            auto t1 = std::chrono::system_clock::now();
            auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
            prevT = t1;

            Eigen::Matrix<double, 3,1> Zk = newPosition.cast<double>();
            kf->stepKF(Zk , static_cast<double>(incT));

            Eigen::Matrix<double, 6,1> Xk = kf->state();
            // std::cout << "Real position->      x:" << Zk[0] << " y:" << Zk[1] << " z:" << Zk[2] << std::endl;
            // std::cout << "Estimated position-> x:" << Xk[0] << " y:" << Xk[2] << " z:" << Xk[4] << std::endl;
            real_x.push_back(Zk[0]);  real_y.push_back(Zk[1]);  real_z.push_back(Zk[2]);
            estim_x.push_back(Xk[0]); estim_y.push_back(Xk[2]); estim_z.push_back(Xk[4]);

            ouputFile << Xk[0] << "," << Xk[2] << "," << Xk[4] << "\n";

            // #ifdef BEAR_HAS_PYTHON
            //     updateVisualization(real_x, real_y, real_z, estim_x, estim_y, estim_z);
            // #endif
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ouputFile.close();

    #ifdef BEAR_HAS_PYTHON
        updateVisualization(real_x, real_y, real_z, estim_x, estim_y, estim_z);
        matplotlibcpp::show();
    #endif

    return 0;
}