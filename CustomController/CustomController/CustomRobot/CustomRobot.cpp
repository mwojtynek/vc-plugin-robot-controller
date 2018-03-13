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

	// KDL CHAIN
	for (int i = 0; i < jointCount; i++) {
		chain.addSegment(Segment(Joint(Joint::RotZ), 
			Frame::DH( DH_Data[i*4 + 0], 
				       d2r(DH_Data[i*4 + 1]), 
				       DH_Data[i*4 + 2], 
				       d2r(DH_Data[i*4 + 3]))
		));
	}

	int len = chain.getNrOfJoints();
	Kin defaultBound;
	defaultBound.j = maxJerk;
	defaultBound.a = maxAcceleration;
	defaultBound.v = maxSpeed;

	bounds = std::vector<Kin>(len, defaultBound);

}

void CustomRobot::setMaxSpeed(double maxSpeed, int joint)
{
	bounds[joint].v = maxSpeed;
}

double CustomRobot::getMaxSpeed(int joint)
{
	return bounds[joint].v;
}

void CustomRobot::setMaxAcceleration(double maxAcceleration, int joint)
{
	bounds[joint].a = maxAcceleration;
}

double CustomRobot::getMaxAcceleration(int joint)
{
	return bounds[joint].a;
}

void CustomRobot::setMaxJerk(double maxJerk, int joint)
{
	bounds[joint].j = maxJerk;
}

double CustomRobot::getMaxJerk(int joint)
{
	return bounds[joint].j;
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


int CustomRobot::IK(const double * frame, double * joints)
{
	return 0;
}
