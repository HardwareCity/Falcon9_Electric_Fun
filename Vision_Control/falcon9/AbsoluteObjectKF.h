#ifndef AbsoluteObjectKF_H_
#define AbsoluteObjectKF_H_

#include <Eigen/Dense>
#include <iostream>

using namespace std;

typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,6,6> Matrix6f;

class AbsoluteObjectKF
{
public:
	AbsoluteObjectKF();
	virtual ~AbsoluteObjectKF();
	void predict(float deltaT, const Matrix6f& Q);
	void update(Eigen::Vector3f position, Eigen::Vector3f vel, float deltaT, const Matrix6f& R);
	void get(Vector6f& state);
	void reset();

	void set(Vector6f& state);
	void set(Vector6f& state, Matrix6f& covar);

	void setPos(const Eigen::Vector3f& pos)
	{
		state(0) = pos(0);
		state(1) = pos(1);
		state(2) = pos(2);
	}

	void setVel(const Eigen::Vector3f& vel)
	{
		state(3) = vel(0);
		state(4) = vel(1);
		state(5) = vel(2);
	}

	Eigen::Vector3f getPos(float deltaT = 0) {
		Eigen::Vector3f ret;
		ret(0) = state(0) + deltaT*state(3);
		ret(1) = state(1) + deltaT*state(4);
		ret(2) = state(2) + deltaT*state(5);
		return ret;
	}

	Eigen::Vector3f getVel() {
		Eigen::Vector3f ret;
		ret(0) = state(3);
		ret(1) = state(4);
		ret(2) = state(5);
		return ret;
	}

	void applyVelocityFactor(float factor) {
		Eigen::Vector3f vel = getVel();
		vel *= factor;
		setVel(vel);
	}

private:
	Matrix6f A, C, covar;
	Eigen::MatrixXf B; // 6x3
	Vector6f state, lastState; // x, y, z, velX, velY, velZ
	bool firstTime;

};

#endif /* AbsoluteObjectKF_H_ */
