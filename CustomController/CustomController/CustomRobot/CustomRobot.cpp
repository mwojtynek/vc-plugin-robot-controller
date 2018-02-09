// CustomRobot.cpp: Definiert die exportierten Funktionen für die DLL-Anwendung.
//




#include <stdio.h>
#include "CustomRobot.h"
#include "Conversions.h"

#include <chainfksolvervel_recursive.hpp>


using namespace KDL;

CustomRobot::CustomRobot(int id, double * DH_Data, int jointCount, double maxSpeed, double maxAcceleration, double maxJerk)
{
	_id = id;
	_maxJerk = maxJerk;
	_maxAcceleration = maxAcceleration;
	_maxSpeed = maxSpeed;

	_lin = NULL;

	// KDL CHAIN
	for (int i = 0; i < jointCount; i++) {
		chain.addSegment(Segment(Joint(Joint::RotZ), 
			Frame::DH( DH_Data[i*4 + 0], 
				       d2r(DH_Data[i*4 + 1]), 
				       DH_Data[i*4 + 2], 
				       d2r(DH_Data[i*4 + 3]))
		));
	}

}

void CustomRobot::setMaxSpeed(double maxSpeed)
{
	_maxSpeed = maxSpeed;
}

double CustomRobot::getMaxSpeed()
{
	return _maxSpeed;
}

void CustomRobot::setMaxAcceleration(double maxAcceleration)
{
	_maxAcceleration = maxAcceleration;
}

double CustomRobot::getMaxAcceleration()
{
	return _maxAcceleration;
}

void CustomRobot::setMaxJerk(double maxJerk)
{
	_maxJerk = maxJerk;
}

double CustomRobot::getMaxJerk()
{
	return _maxJerk;
}

int CustomRobot::FK(const double * joints, double * frame)
{
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	KDL::JntArray jointpositions(chain.getNrOfJoints());
	doubleToJnt(jointpositions, joints, d2r);

	KDL::Frame solutionFrame;
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, solutionFrame);
	if (kinematics_status < 0) {
		return -1;
	}
	frameToDouble(frame, solutionFrame);
	return 0;
}

int CustomRobot::SpeedFK(const double * joints, const double * jointsDot, double * twist) {
	ChainFkSolverVel_recursive fksolver = ChainFkSolverVel_recursive(chain);
	int jntCount = chain.getNrOfJoints();
	JntArray q(jntCount);
	JntArray qdot(jntCount);
	doubleToJnt(q, joints, d2r);
	doubleToJnt(qdot, jointsDot, d2r);
	JntArrayVel qVel(q, qdot);
	FrameVel frameVel;
	if (fksolver.JntToCart(qVel, frameVel) < 0) {
		return -1;
	}
	vectorToDouble(twist, frameVel.p.v);
	vectorToDouble(twist + 3, frameVel.M.w);
	return 0;
}

int CustomRobot::LIN(const double * start, const double * stop)
{
	//und andere...
	if (_lin != NULL) {
		return -1;
	}

	_lin = new LINInterpolater(start, stop, _maxSpeed, _maxAcceleration);
	return 0;
}



int CustomRobot::step(double * output)
{
	if (_lin == NULL) {
		return -1;
	}
	int result = _lin->getStep(output);
	switch (result) {
	case ReflexxesAPI::RMLResultValue::RML_WORKING: break;
	case ReflexxesAPI::RMLResultValue::RML_FINAL_STATE_REACHED:
		delete _lin;
		_lin = NULL;
		return 1;
	break;
	default: printf("Some problem with Reflexxes: Error Code %d\n", result); 
		return -1;
	break;
	}
	return 0;

}

int CustomRobot::IK(const double * frame, double * joints)
{
	return 0;
}
