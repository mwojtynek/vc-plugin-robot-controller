#pragma once

#include <ReflexxesAPI.h>

class PTPInterpolater {
private:
	double * _startJoints;
	double * _goalJoints;
	
	double * _direction;
	double _pathLength;

	double _maxSpeed;
	double _maxAccel;

	double _cycleTime;
	int _jntCount;

	ReflexxesAPI *RML;
	RMLPositionInputParameters *IP;
	RMLPositionOutputParameters *OP;
	RMLPositionFlags Flags;

public:
	PTPInterpolater(const double *startJoints, const double *goaljoints, int jointCount, double maxSpeed = 200.0, double maxAcc = 1600.0, double cycleTime = 33);
	~PTPInterpolater();

	int getStep(double * joints);

	void setMaxSpeed(double maxSpeed);
	double getMaxSpeed();

	double getMaxAccel();

};