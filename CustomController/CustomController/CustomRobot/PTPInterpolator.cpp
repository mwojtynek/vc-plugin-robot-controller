#include "PTPInterpolator.h"

PTPInterpolater::PTPInterpolater(const double * startJoints, const double  * goalJoints, int jointCount, double maxSpeed, double maxAcc, double cycleTime)
{
	_jntCount = jointCount;
	_pathLength = 0.0;
	_startJoints = new double[_jntCount];
	_goalJoints = new double[_jntCount];
	_direction = new double[_jntCount];
	for (int i = 0; i < _jntCount; i++) {
		_startJoints[i] = startJoints[i];
		_goalJoints[i] = goalJoints[i];
		_direction[i] = goalJoints[i] - startJoints[i];
	}
	
	//calc Pathlength:


	
	
	
	for (int i = 0; i < 6; i++) {
		_direction[i] /= _pathLength;
	}

	_maxSpeed = maxSpeed;
	_maxAccel = maxAcc;

	_cycleTime = cycleTime;

	RML = new ReflexxesAPI(1, _cycleTime);
	IP = new RMLPositionInputParameters(1);
	OP = new RMLPositionOutputParameters(1);

	//for (int i = 0; i < _jntCount; i++) {
		IP->CurrentPositionVector->VecData[0] = 0.0;
		IP->CurrentVelocityVector->VecData[0] = 0.0;

		IP->MaxVelocityVector->VecData[0] = _maxSpeed;
		IP->MaxAccelerationVector->VecData[0] = _maxAccel;

		IP->TargetVelocityVector->VecData[0] = 0.0;
		IP->TargetPositionVector->VecData[0] = _pathLength;

		IP->SelectionVector->VecData[0] = true;
	//}

}

PTPInterpolater::~PTPInterpolater()
{
	delete[] _startJoints;
	delete[] _goalJoints;
	delete RML;
	delete IP;
	delete OP;
}

int PTPInterpolater::getStep(double * joints)
{
	int ResultValue = RML->RMLPosition(*IP
		, OP
		, Flags);

	*IP->CurrentPositionVector = *OP->NewPositionVector;
	*IP->CurrentVelocityVector = *OP->NewVelocityVector;

	double _pathStroke = OP->NewPositionVector->VecData[0];

	for (int i = 0; i < 6; i++) {
		joints[i] = _startJoints[i] + _direction[i] * _pathStroke;
	}

	return ResultValue;


}

void PTPInterpolater::setMaxSpeed(double maxSpeed)
{
	_maxSpeed = maxSpeed;
}

double PTPInterpolater::getMaxSpeed()
{
	return _maxSpeed;
}

double PTPInterpolater::getMaxAccel()
{
	return _maxAccel;
}
