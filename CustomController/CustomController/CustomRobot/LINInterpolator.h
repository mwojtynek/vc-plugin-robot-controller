#pragma once

// UNITS IN MM

#include <ReflexxesAPI.h>

class LINInterpolater {
private:
	double _startFrame[6];
	double _goalFrame[6];

	double _direction[6];
	double _pathLength;

	double _maxSpeed;
	double _maxAccel;

	double _cycleTime;

	ReflexxesAPI *RML;
	RMLPositionInputParameters *IP;
	RMLPositionOutputParameters *OP;
	RMLPositionFlags Flags;

public:
	LINInterpolater(const double startFrame[6], const double goalFrame[6], double maxSpeed = 200.0, double maxAcc = 1600.0, double cycleTime = 33);
	~LINInterpolater();

	int getStep(double * frame);

	void setMaxSpeed(double maxSpeed);
	double getMaxSpeed();

	double getMaxAccel();

};
