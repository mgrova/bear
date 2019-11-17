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

#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>

#include <thread>
#include <chrono>
#include<bits/stdc++.h>

using namespace std;
using namespace mico ;


int WORLD_SIZE = 5;

inline double gauss(const double & _nu, const double & _sigma) { 
	std::random_device rd; 
	std::mt19937 gen(rd()); 

	std::normal_distribution<> d(_nu, _sigma); 

	return d(gen); 
} 

float LANDMARK_DIST = 5;

const std::vector<Eigen::Vector3f> LANDMARKS = {
	{LANDMARK_DIST,LANDMARK_DIST,LANDMARK_DIST},
	{LANDMARK_DIST,LANDMARK_DIST,-LANDMARK_DIST},
	{LANDMARK_DIST,-LANDMARK_DIST,-LANDMARK_DIST},
	{-LANDMARK_DIST,-LANDMARK_DIST,-LANDMARK_DIST},
	{LANDMARK_DIST,-LANDMARK_DIST,LANDMARK_DIST},
	{-LANDMARK_DIST,-LANDMARK_DIST,LANDMARK_DIST},
	{-LANDMARK_DIST,LANDMARK_DIST,LANDMARK_DIST},
	{-LANDMARK_DIST,LANDMARK_DIST,-LANDMARK_DIST}
};

const int N_LANDMARKS =  8;

struct ObservationData{
	std::vector<float>distLandmarks;
};

class ParticleRobot : public ParticleInterface<ObservationData>{
public:

	int sign(float _x){
		return _x < 0? -1:1;
	}

	ParticleRobot() {
		mPosition[0] = gauss(0,2);
		mPosition[1] = gauss(0,2);
		mPosition[2] = gauss(0,2);
		mSpeed[0] = gauss(0,1);
		mSpeed[1] = gauss(0,1);
		mSpeed[2] = gauss(0,1);

		mSpeed[0] = fabs(mSpeed[0]) > 0.5? sign(mSpeed[0])*0.5 : mSpeed[0];
		mSpeed[1] = fabs(mSpeed[1]) > 0.5? sign(mSpeed[1])*0.5 : mSpeed[1];
		mSpeed[2] = fabs(mSpeed[2]) > 0.5? sign(mSpeed[2])*0.5 : mSpeed[2];
	}
	void simulate() { 
		mPosition[0] += mSpeed[0]*0.06;
		mPosition[1] += mSpeed[1]*0.06;
		mPosition[2] += mSpeed[2]*0.06;


		mSpeed[0] += gauss(0,0.2);
		mSpeed[1] += gauss(0,0.2);
		mSpeed[2] += gauss(0,0.2);

		mSpeed[0] = fabs(mSpeed[0]) > 1.0? sign(mSpeed[0])*1.0 : mSpeed[0];
		mSpeed[1] = fabs(mSpeed[1]) > 1.0? sign(mSpeed[1])*1.0 : mSpeed[1];
		mSpeed[2] = fabs(mSpeed[2]) > 1.0? sign(mSpeed[2])*1.0 : mSpeed[2];

		if(mPosition[0] > WORLD_SIZE)
			mPosition[0] -= WORLD_SIZE*2;
		else if (mPosition[0] < -WORLD_SIZE)
			mPosition[0] += WORLD_SIZE*2;

		if(mPosition[1] > WORLD_SIZE)
			mPosition[1] -= WORLD_SIZE*2;
		else if (mPosition[1] < -WORLD_SIZE)
			mPosition[1] += WORLD_SIZE*2;

		if(mPosition[2] > WORLD_SIZE)
			mPosition[2] -= WORLD_SIZE*2;
		else if (mPosition[2] < -WORLD_SIZE)
			mPosition[2] += WORLD_SIZE*2;

	};

	void simulateReal() { 
		mPosition[0] += mSpeed[0]*0.06;
		mPosition[1] += mSpeed[1]*0.06;
		mPosition[2] += mSpeed[2]*0.06;

		if(mPosition[0] > WORLD_SIZE)
			mPosition[0] -= WORLD_SIZE*2;
		else if (mPosition[0] < -WORLD_SIZE)
			mPosition[0] += WORLD_SIZE*2;

		if(mPosition[1] > WORLD_SIZE)
			mPosition[1] -= WORLD_SIZE*2;
		else if (mPosition[1] < -WORLD_SIZE)
			mPosition[1] += WORLD_SIZE*2;

		if(mPosition[2] > WORLD_SIZE)
			mPosition[2] -= WORLD_SIZE*2;
		else if (mPosition[2] < -WORLD_SIZE)
			mPosition[2] += WORLD_SIZE*2;

	};

	ObservationData observation(){
		ObservationData data;
		for(unsigned i = 0; i < LANDMARKS.size(); i++){
			data.distLandmarks.push_back((mPosition - LANDMARKS[i]).norm() + gauss(0,0.2));
		}
		
		return data;
	}

	double computeWeight(ObservationData &_observation) {  
		ObservationData fakeData = observation();
		ObservationData realData = _observation;

		std::vector<float> distanceWeights;


		double score = 1;
		for(unsigned i = 0; i < fakeData.distLandmarks.size(); i++){
			score *= 1 - fabs(fakeData.distLandmarks[i] - realData.distLandmarks[i])/(WORLD_SIZE*2);
		}

		return score;
	};

	Eigen::Vector3f position(){ return mPosition; }

private:
	Eigen::Vector3f mPosition;	// x, y, z
	Eigen::Vector3f mSpeed;	// x, y, z

};

//---------------------------------------------------------------------------------------------------------------------
Eigen::Vector3f mediumState(std::vector<ParticleRobot> _particles);
void particleFilterCPU();

//---------------------------------------------------------------------------------------------------------------------
int main(void){
	
	particleFilterCPU();

	std::cout << "Finished" << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3f mediumState(std::vector<ParticleRobot> _particles){
	Eigen::Vector3f position = {0.0, 0.0, 0.0};
	for (unsigned i = 0; i < _particles.size(); i++){
		Eigen::Vector3f pos = _particles[i].position();
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

	pcl::visualization::PCLVisualizer viewer ("viewer");

	viewer.addCoordinateSystem(0.2);
	
	for (;;) {
		Eigen::Vector3f medState = mediumState(filter.particles());
		Eigen::Vector3f realState = robot.position();
		std::cout << "-------------------------------------------------------------------" << std::endl;
		std::cout << "Real state. X:" << realState[0] << " ; Y: " << realState[1] << " ; Z: " << realState[2] << std::endl;
		std::cout << "Promediate state. X:" << medState[0] << " ; Y: " << medState[1] << " ; Z: " << medState[2] << std::endl;
		std::cout << "Error. X:" << fabs(realState[0] - medState[0]) << " ; Y: " << fabs(realState[1] - medState[1]) << " ; Z: " << fabs(realState[2] -  medState[2]) << std::endl;

		viewer.removeAllShapes();
		viewer.removeAllPointClouds();
		// Draw landmarks
		int counter = 0;
		for(auto &lm: LANDMARKS){
			viewer.addSphere(pcl::PointXYZ(lm[0],lm[1],lm[2]),0.15, 1,0,0,"sphere"+std::to_string(counter++));
		}

		// Draw particles
		pcl::PointCloud<pcl::PointXYZRGB> particles;
		for(auto &particle: filter.particles()){
			pcl::PointXYZRGB p (0,0,255);

			Eigen::Vector3f pos = particle.position();
			
			p.x = pos[0];
			p.y = pos[1];
			p.z = pos[2];
			particles.push_back(p);
		}
		viewer.addPointCloud(particles.makeShared(), "cloud_particles");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_particles");
			

		// Draw robot
		viewer.addSphere(pcl::PointXYZ(realState[0],realState[1],realState[2]),0.2, 1,0,0,"sphere"+std::to_string(counter++));

		// Estimate
		viewer.addSphere(pcl::PointXYZ(medState[0],medState[1],medState[2]),0.2, 0,1,0,"sphere"+std::to_string(counter++));


		viewer.spinOnce(30);
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

		robot.simulateReal();
		ObservationData obs = robot.observation();
		filter.step(obs);

	}
}