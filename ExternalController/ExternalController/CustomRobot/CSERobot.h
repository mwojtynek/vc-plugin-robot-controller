#pragma once

#include "RobotKinematicManager.h"
#include "RobotMotionManager.h"

#include "KinematicVector.h"

/**
 Class is written as easy as possible. Easy means the marshalling of the function is made quite easy by using primitiv data structures.

 */

class CSERobot {
private:
	RobotKinematicManager *kinematic;
	RobotMotionManager *motion;

public:
	//CSERobot(int id, double *DH_Data, int jointCoint, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);
	CSERobot(std::string urdf_path);
	~CSERobot();
	
	//initStuff
	int reInit(double * pos, double cycleTime);
	

	//controller
	//Argumente müssen definiert werden!
	//int setTask()
	int addTask(Task *task);
	int halt();

	int nextCycle(double * pos, double * vel, double * acc);
	
	//low-level kinematic
	int FK(const double * joints, double * frame);
	int SpeedFK(const double * joints, const double * jointsDot, double * twist);
		
	int IK(const double * frame, double * joints);

};