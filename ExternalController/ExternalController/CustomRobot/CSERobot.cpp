// CustomRobot.cpp: Definiert die exportierten Funktionen für die DLL-Anwendung.
//

#include "CSERobot.h"

#include <kdl\jntarrayacc.hpp>

#include "Conversions.h"

CSERobot::CSERobot(std::string urdf_path) {
	kinematic = new RobotKinematicManager(urdf_path);
	motion = new RobotMotionManager(kinematic, 0.1);

	int dof = kinematic->getDOF();
	
	KDL::JntArrayAcc state(dof);
	state.q(0) = d2r(90.0);

	motion->init(&state, 0.1);
}

CSERobot::~CSERobot() {
	delete motion;
	delete kinematic;
}

int CSERobot::reInit(double * pos, double cycleTime)
{
	KDL::JntArrayAcc state(this->kinematic->getDOF());
	memcpy(state.q.data.data(), pos, sizeof(double) * this->kinematic->getDOF());
	motion->init(&state, cycleTime);
	return 0;
}

int CSERobot::addTask(Task * task)
{
	motion->addTask(task);
	return 0;
}

int CSERobot::halt()
{
	return 0;
}

int CSERobot::nextCycle(double * pos, double * vel, double * acc)
{
	//TODO entry point fuer alle Module!
	
	int size = this->kinematic->getDOF();
	KDL::JntArrayAcc state(size);
	int ret = motion->nextCycle(&state);
	
	int memsize = size * sizeof(double);

	memcpy(pos, state.q.data.data(), memsize);
	memcpy(vel, state.qdot.data.data(), memsize);
	memcpy(acc, state.qdotdot.data.data(), memsize);
	
	return ret;
}

int CSERobot::FK(const double * joints, double * frame)
{
	KDL::Frame kdlFrame;
	KDL::JntArray arr(this->kinematic->getDOF());

	Marshal::doubleToJnt(arr, joints, d2r);

	int ret = kinematic->FK(&arr, &kdlFrame);

	Marshal::frameToDouble(frame, kdlFrame);

	return ret;
}

int CSERobot::SpeedFK(const double * joints, const double * jointsDot, double * twist) {
//	return kinematic->SpeedFK(joints, jointsDot, twist);
}

int CSERobot::IK(const double * frame, double * joints)
{
	KDL::Frame kdlFrame;
	KDL::JntArray arr;

	Marshal::doubleToFrame(kdlFrame, frame);

	int ret = kinematic->IK(&kdlFrame, NULL, &arr);

	Marshal::jntToDouble(joints, arr, r2d);

	return ret;
}
