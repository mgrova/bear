//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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
#include <string>

#include <mico/base/state_filtering/ParticleFilterCPU.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include<bits/stdc++.h>

using namespace std;
using namespace mico;


int WORLD_SIZE = 300;

inline double gauss(const double & _nu, const double & _sigma) { 
	std::random_device rd; 
	std::mt19937 gen(rd()); 

	std::normal_distribution<> d(_nu, _sigma); 

	return d(gen); 
} 

struct ObservationData{
	double distanceOrigin = 0;
	double angle = 0;
};

class ParticleRobot : public ParticleInterface<ObservationData>{
public:
	ParticleRobot() {
		mPosition[0] = gauss(WORLD_SIZE/2,25);
		mPosition[1] = gauss(WORLD_SIZE/2,25);
		mPosition[2] = gauss(0,1);
	}
	void simulate() { 
		mPosition[0] += 1*cos(mPosition[2]) + gauss(0,0.1);
		mPosition[1] += 1*sin(mPosition[2]) + gauss(0,0.1);
		mPosition[2] += gauss(0,0.05);

		if(mPosition[0] > WORLD_SIZE)
			mPosition[0] = fmod(mPosition[0], WORLD_SIZE);
		else if (mPosition[0] < 0)
			mPosition[0] += WORLD_SIZE;
		
		if(mPosition[1] > WORLD_SIZE)
			mPosition[1] = fmod(mPosition[1], WORLD_SIZE);
		else if (mPosition[1] < 0)
			mPosition[1] += WORLD_SIZE;

		if(mPosition[2] > M_PI*2)
			mPosition[2] = fmod(mPosition[2], M_PI*2);

			
	};

	ObservationData observation(){
		double xNoise = mPosition[0] + gauss(0,0.1);
		double yNoise = mPosition[1] + gauss(0,0.1);

		ObservationData data;
		data.distanceOrigin = sqrt(xNoise*xNoise + yNoise*yNoise);
		data.angle = mPosition[2];

		return data;
	}

	double computeWeight(ObservationData &_observation) {  
		ObservationData fakeData = observation();
		ObservationData realData = _observation;

		double weightDistance = 1 - fabs(fakeData.distanceOrigin-realData.distanceOrigin)/WORLD_SIZE/2;
		double weightAngle = 1 - fabs(fakeData.angle - realData.angle)/(M_PI*2);

		return weightAngle*weightDistance;
	};

	std::array<double, 3> position(){ return mPosition; }
	operator std::array<double, 3>(){ return mPosition; }

private:
	std::array<double, 3> mPosition;	// x, y, ori

};

//---------------------------------------------------------------------------------------------------------------------
std::array<double, 3> mediumState(std::vector<ParticleRobot> _particles);
void particleFilterCPU();

//---------------------------------------------------------------------------------------------------------------------
int main(void){
	
	particleFilterCPU();

	std::cout << "Finished" << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------

std::array<double, 3> mediumState(std::vector<ParticleRobot> _particles){
	std::array<double, 3> position = {0.0, 0.0, 0.0};
	for (unsigned i = 0; i < _particles.size(); i++){
		std::array<double, 3> pos = _particles[i];
		position[0] += pos[0];
		position[1] += pos[1];
		position[2] += pos[2];
	}
	position[0] /= _particles.size();
	position[1] /= _particles.size();
	position[2] /= _particles.size();

	return position;
}

void particleFilterCPU() {

	ParticleFilterCPU<ParticleRobot, ObservationData> filter(1000);
	filter.init();

	ParticleRobot robot;
	  double time = 0.0;

	float cScaleFactor=1;

	cv::namedWindow("display", CV_WINDOW_FREERATIO);
	
	while(cv::waitKey(30)!='p'){
	}
	
	for (;;) {
		std::array<double, 3> medState = mediumState(filter.particles());
		std::array<double, 3> realState = robot.position();
		std::cout << "-------------------------------------------------------------------" << std::endl;
		std::cout << "Real state. X:" << realState[0] << " ; Y: " << realState[1] << " ; Ori: " << realState[2] << std::endl;
		std::cout << "Promediate state. X:" << medState[0] << " ; Y: " << medState[1] << " ; Ori: " << medState[2] << std::endl;


		cv::Mat display(WORLD_SIZE, WORLD_SIZE, CV_8UC3, cv::Scalar(0,0,0));
		// Draw landmarks
		
		cv::circle(display, cv::Point2i(WORLD_SIZE/2, WORLD_SIZE/2), 3, cv::Scalar(0,0,255), 3);

		// Draw particles
		for(auto &particle: filter.particles()){
			std::array<double, 3> pos = particle;
			cv::arrowedLine(display, 	cv::Point2i(pos[0]*cScaleFactor, pos[1]*cScaleFactor), 
										cv::Point2i(pos[0]*cScaleFactor, pos[1]*cScaleFactor) + cv::Point2i(cos(realState[2])*5*cScaleFactor, sin(realState[2])*5*cScaleFactor), 
										cv::Scalar(255,0,0), 1);
		}

		// Draw robot
		cv::arrowedLine(display, 	cv::Point2i(realState[0]*cScaleFactor, realState[1]*cScaleFactor), 
									cv::Point2i(realState[0]*cScaleFactor, realState[1]*cScaleFactor) + cv::Point2i(cos(realState[2])*5*cScaleFactor, sin(realState[2])*5*cScaleFactor), 
									cv::Scalar(0,0,255), 2);

		// Draw robot
		cv::arrowedLine(display, 	cv::Point2i(medState[0]*cScaleFactor, medState[1]*cScaleFactor), 
									cv::Point2i(medState[0]*cScaleFactor, medState[1]*cScaleFactor) + cv::Point2i(cos(medState[2])*5*cScaleFactor, sin(medState[2])*5*cScaleFactor), 
									cv::Scalar(0,255,0), 2);


		cv::imshow("display", display);
		cv::waitKey(3);

		robot.simulate();
		ObservationData obs = robot.observation();
		filter.step(obs);

	}
}