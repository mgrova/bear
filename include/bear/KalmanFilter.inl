//---------------------------------------------------------------------------------------------------------------------
//  EKF ABSTRACT CLASS (MICO)
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


namespace bear {
	
	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	KalmanFilter<_type, _D1, _D2>::KalmanFilter(){
		
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	void KalmanFilter<_type, _D1, _D2>::setupKF( const Eigen::Matrix<_type, _D1, _D1 > _Q , 
					            				 const Eigen::Matrix<_type, _D2, _D2 > _R , 
					            				 const Eigen::Matrix<_type, _D1,  1  > _x0,
												 const Eigen::Matrix<_type, _D2, _D1 > _C ){
		Q_ = _Q;
		R_ = _R;
		Xak_ = _x0;
		Xfk_ = _x0;
		C_ = _C;
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	Eigen::Matrix<_type , _D1 , 1> KalmanFilter<_type, _D1, _D2>::state() const{
		return Xak_;
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	void KalmanFilter<_type, _D1, _D2>::stepKF(const Eigen::Matrix<_type, _D2, 1 > & _Zk, const _type _incT){
		predictionStep(_incT);
		updateStep(_Zk);
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	void KalmanFilter<_type, _D1, _D2>::stepKF(const _type _incT){
		predictionStep(_incT);
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	void KalmanFilter<_type, _D1, _D2>::predictionStep(const _type _incT){
        updateA(_incT);

		Xfk_ = A_ * Xak_ ;//+ B_; 	
		Pfk_ = A_ * Pak_ * A_.transpose() + Q_;
	}

	//-----------------------------------------------------------------------------
    template<typename _type, int _D1, int _D2>
	void KalmanFilter<_type, _D1, _D2>::updateStep(const Eigen::Matrix<_type, _D2, 1 >&_Zk){

		K_ = Pfk_ * C_.transpose() * ((C_ * Pfk_ * C_.transpose() + R_).inverse());	
		Xak_ = Xfk_ + K_ * (_Zk - C_ * Xfk_);

		Eigen::Matrix<_type, _D1, _D1> I; I.setIdentity();
		Pak_ = (I - K_ * C_) * Pfk_;

	}
	
}
