#include "Conversions.h"

#define _USE_MATH_DEFINES
#include <math.h>


double d2r(const double deg)
{
	return M_PI * deg / 180.0;
}

void vectorToDouble(double * dVector, KDL::Vector vector) {
	dVector[0] = vector.x();
	dVector[1] = vector.y();
	dVector[2] = vector.z();
}

void jntToDouble(double * dJoints, KDL::JntArray joints, double(*conv)(double))
{
	for (int i = 0; i < joints.rows(); i++) {
		if (conv == NULL) {
			dJoints[i] = joints(i);
		}
		else {
			dJoints[i] = conv(joints(i));
		}
	}
}

void twistToDouble(double * dTwist, KDL::Twist twist)
{
	vectorToDouble(dTwist    , twist.vel);
	vectorToDouble(dTwist + 3, twist.rot);
}

void doubleToFrame(KDL::Frame & frame, const double * dFrame)
{
	for (int i = 0; i < 9; i++) {
		frame.M.data[i] = dFrame[i];
	}
	doubleToVector(frame.p, dFrame + 9);
}

void doubleToVector(KDL::Vector & vector, const double * dVector)
{
	vector.x(dVector[0]);
	vector.y(dVector[1]);
	vector.z(dVector[2]);
}

void doubleToJnt(KDL::JntArray & joints, const double * dJoints, double(*conv)(double))
{
	for (int i = 0; i < joints.rows(); i++) {
		if (conv == NULL) {
			joints(i) = dJoints[i];
		}
		else {
			joints(i) = conv(dJoints[i]);
		}
	}
}

void doubleToTwist(KDL::Twist & twist, const double * dTwist)
{	
	doubleToVector(twist.vel, dTwist);
	doubleToVector(twist.rot, dTwist + 3);
}

void frameToDouble(double * dFrame, KDL::Frame frame) {
	for (int i = 0; i < 9; i++) {
		dFrame[i] = frame.M.data[i];
	}
	vectorToDouble(dFrame + 9, frame.p);
}