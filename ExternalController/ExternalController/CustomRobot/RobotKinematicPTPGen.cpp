#include "RobotKinematicPTPGen.h"

#include <kdl\jntarray.hpp>

using namespace CSEController;

RobotKinematicPTPGen::RobotKinematicPTPGen(RobotKinematicManager * kinematic, double cycleTime)
{
	this->kinematic = kinematic;
	int DOF = this->kinematic->getDOF();

	this->RML = new ReflexxesAPI(DOF,cycleTime);
	this->IP = new RMLPositionInputParameters(DOF);
	this->OP = new RMLPositionOutputParameters(DOF);
	this->Flags = new RMLPositionFlags();
}

RobotKinematicPTPGen::~RobotKinematicPTPGen()
{
	delete  RML;
	delete  IP;
	delete  OP;
	delete Flags;
}

int RobotKinematicPTPGen::init(KDL::JntArrayAcc *start, Task *task)
{

	KDL::JntArray target = task->target->getConfig(this->kinematic);

	for (int i = 0; i < this->kinematic->getDOF(); i++) {
		IP->CurrentPositionVector->VecData[i] = (start->q)(i);
		IP->CurrentVelocityVector->VecData[i] = (start->qdot)(i);
		IP->CurrentAccelerationVector->VecData[i] = (start->qdotdot)(i);

		IP->MaxVelocityVector->VecData[i] = 10.0;
		IP->MaxAccelerationVector->VecData[i] = 1.0;
		IP->MaxJerkVector->VecData[i] = 1.0;

		IP->TargetPositionVector->VecData[i] = target(i);
		IP->TargetVelocityVector->VecData[i] = 0.0;
		
		IP->SelectionVector->VecData[i] = true;
	}
	if (IP->CheckForValidity()) {
		return 0;
	}
	else {
		return -1;
	}
}

int RobotKinematicPTPGen::nextCycle(KDL::JntArrayAcc *nextVector)
{
	int ret = RML->RMLPosition(*IP, OP, *Flags);
	
	*IP->CurrentPositionVector = *OP->NewPositionVector;
	*IP->CurrentVelocityVector = *OP->NewVelocityVector;
	*IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

	int DOF = this->kinematic->getDOF();

	memcpy(nextVector->q.data.data(), OP->NewPositionVector->VecData, DOF * sizeof(double));
	memcpy(nextVector->qdot.data.data(), OP->NewVelocityVector->VecData, DOF * sizeof(double));
	memcpy(nextVector->qdotdot.data.data(), OP->NewAccelerationVector->VecData, DOF * sizeof(double));
	
	return ret;
}
