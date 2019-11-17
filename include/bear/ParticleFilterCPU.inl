//---------------------------------------------------------------------------------------------------------------------
//  BEAR
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

namespace bear {
	
	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::step(ObservationData_ &_observation) {
		simulate();
		calcWeight(_observation);
		resample();
	}

	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::step() {
		simulate();
		for (unsigned i = 0; i < mNuParticles; i++) {
			mParticles[i].setWeight(1.0);
		}
		resample();
	}

	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::init(){
		for (unsigned i = 0; i < mNuParticles; i++){
			mParticles.push_back(ParticleType_());
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::simulate() {
		for (unsigned i = 0; i < mNuParticles; i ++) {
			mParticles[i].simulate();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::calcWeight(ObservationData_ &_observation) {
		for (unsigned i = 0; i < mNuParticles; i++) {
			mParticles[i].updateWeight(_observation);
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template <typename ParticleType_, typename ObservationData_>
	void ParticleFilterCPU<ParticleType_, ObservationData_>::resample() {
		std::vector<ParticleType_> newParticles;
		double beta = 0.0;
		unsigned index = unsigned(double(rand()) / RAND_MAX * mNuParticles);
		double maxWeight = 0.0;

		for (unsigned i = 0; i < mNuParticles; i++) {
			if (mParticles[i].weight() > maxWeight)
				maxWeight = mParticles[i].weight();
		}

		for (unsigned i = 0; i < mNuParticles; i++) {
			beta += double(rand()) / RAND_MAX * 2.0 * maxWeight;
			while (beta > mParticles[index].weight()) {
				beta -= mParticles[index].weight();
				index = (index + 1) % mNuParticles;
			}
			newParticles.push_back(mParticles[index]);
		}
		
		mParticles = newParticles;
	}
}
