#include "LINInterpolator.h"

LINInterpolater::LINInterpolater(const double startFrame[6], const double goalFrame[6], double maxSpeed, double maxAcc, double cycleTime)
{
	_pathLength = 0.0;
	for (int i = 0; i < 6; i++) {
		_startFrame[i] = startFrame[i];
		_goalFrame[i] = goalFrame[i];
		_direction[i] = goalFrame[i] - startFrame[i];
		if (i < 3) {
			_pathLength += _direction[i] * _direction[i];
		}
	}
	_pathLength = sqrt(_pathLength);
	for (int i = 0; i < 6; i++) {
		_direction[i] /= _pathLength;
	}
	
	_maxSpeed = maxSpeed;
	_maxAccel = maxAcc;

	_cycleTime = cycleTime;

	RML = new ReflexxesAPI(1, _cycleTime);
	IP = new RMLPositionInputParameters(1);
	OP = new RMLPositionOutputParameters(1);

	IP->CurrentPositionVector->VecData[0] = 0.0;
	IP->CurrentVelocityVector->VecData[0] = 0.0;
	
	IP->MaxVelocityVector->VecData[0] = _maxSpeed;
	IP->MaxAccelerationVector->VecData[0] = _maxAccel;

	IP->TargetVelocityVector->VecData[0] = 0.0;
	IP->TargetPositionVector->VecData[0] = _pathLength;

	IP->SelectionVector->VecData[0] = true;
	
}

LINInterpolater::~LINInterpolater()
{
	delete RML;
	delete IP;
	delete OP;
}

int LINInterpolater::getStep(double * frame)
{
	int ResultValue = RML->RMLPosition(*IP
		, OP
		, Flags);

	*IP->CurrentPositionVector = *OP->NewPositionVector;
	*IP->CurrentVelocityVector = *OP->NewVelocityVector;

	double _pathStroke = OP->NewPositionVector->VecData[0];

	for (int i = 0; i < 6; i++) {
		frame[i] = _startFrame[i] + _direction[i] * _pathStroke;
	}

	return ResultValue;

	
}

void LINInterpolater::setMaxSpeed(double maxSpeed)
{
	_maxSpeed = maxSpeed;
}

double LINInterpolater::getMaxSpeed()
{
	return _maxSpeed;
}

double LINInterpolater::getMaxAccel()
{
	return _maxAccel;
}
