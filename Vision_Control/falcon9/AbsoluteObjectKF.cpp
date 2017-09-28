#include "AbsoluteObjectKF.h"

AbsoluteObjectKF::AbsoluteObjectKF() : B(6,3)
{
	firstTime = true;
	A.setZero(); // changed in runtime, based on the motion of the robot

	B.setZero();
	C.setIdentity();

	reset();
}

AbsoluteObjectKF::~AbsoluteObjectKF()
{
}

void AbsoluteObjectKF::predict(float deltaT, const Matrix6f& Q)
{
	A.setIdentity();
	A(0,3) = deltaT;
	A(1,4) = deltaT;
	A(2,5) = deltaT;
	
	Eigen::Vector3f control;
	control.setZero();

	state = A*state + B*control;
	covar = A*covar*A.transpose() + Q;

	/*
		cout << "KF control " << endl << control << endl
			 << "KF R " << endl << R << endl
			 << "KF State " << endl << state << endl
			 << "KF Covar " << endl << covar << endl;
	 */
}

void AbsoluteObjectKF::update(Eigen::Vector3f position, Eigen::Vector3f velocity, float deltaT, const Matrix6f& R)
{
	Vector6f measure;
	measure << position(0), position(1), position(2), velocity(0), velocity(1), velocity(2);

//	cout << "KF velX " << measure(2) << " velY " << measure(3) << endl;

	Matrix6f K;
	Vector6f residual;

	if(firstTime)
	{
		//cout << "KF firstTime " << endl;
		state = (measure);
		covar = R;
		firstTime = false;
	}
	else
	{
		K = covar*C.transpose()*(C*covar*C.transpose() + R).inverse();

		residual = (measure - C*state);
		state = state + K*residual;
		covar = (Matrix6f::Identity() - K*C)*covar;
	}

	lastState = state;
}

void AbsoluteObjectKF::get(Vector6f& state)
{
	state = this->state;
}

void AbsoluteObjectKF::reset()
{
	state << 0, 0, 0, 0, 0 ,0;
	lastState << 0, 0, 0, 0, 0, 0;
	covar << 999, 0, 0, 0, 0, 0,
			 0, 999, 0, 0, 0, 0,
			 0, 0, 999, 0, 0, 0,
			 0, 0, 0, 1, 0, 0,
			 0, 0, 0, 0, 1, 0,
			 0, 0, 0, 0, 0, 1;
	firstTime = true;
}

void AbsoluteObjectKF::set(Vector6f& state)
{
	this->state.head<6>() = state.head<6>();
}

void AbsoluteObjectKF::set(Vector6f& state, Matrix6f& covar)
{
	this->state.head<6>() = state.head<6>();
	this->covar.block<6,6>(0,0) = covar.block<6,6>(0,0);
}
