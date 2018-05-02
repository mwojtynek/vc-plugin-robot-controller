#include "RobotKinematicLINGen.h"

RobotKinematicLINGen::RobotKinematicLINGen(RobotKinematicManager * kinematic, double cycleTime)
{
	this->kinematic = kinematic;

	this->RML = new ReflexxesAPI(6, cycleTime);
	this->IP = new RMLPositionInputParameters(6);
	this->OP = new RMLPositionOutputParameters(6);
	this->Flags = new RMLPositionFlags();
}

RobotKinematicLINGen::~RobotKinematicLINGen()
{
	delete  RML;
	delete  IP;
	delete  OP;
	delete Flags;
}

int RobotKinematicLINGen::init(KDL::JntArrayAcc * start, Task *task)
{
	// start is in Joints: q(start), qdot(start), qdotdot(start)
	// Calc x(start), xdot(start), xdotdot(start)
	// task enthält x(ende), xdot(ende)
	// task enthält kartesischen Limit

	// Erste Annahme bisher: Vel = 0 , Acc = 0
	// sonst:
	// q = IK (x)
	// qdot = J^-1(q) * xdot 
	// qdotdot = J^-1(q) * ( xdotdot - Jdot(q,qdot) * qdot) 
	// -> RoboticKinematicManager
	
	if (task->type != TaskType::LIN) {
		throw - 1;
	}

	//KDL::Frame target = task->target->getFrame();

	double joints[2];
	task->target->getConfig(joints);
	KDL::JntArray arr(2);
	for (int i = 0; i < 2; i++) {
		arr(i) = joints[i];
	}

	KDL::Frame target;
	this->kinematic->FK(&arr, &target);

	KDL::Frame frame;
	this->kinematic->FK(&start->q, &frame);
	
	double rot[3];
	frame.M.GetRPY(rot[0], rot[1], rot[2]);

	double rotTarget[3];
	target.M.GetRPY(rotTarget[0], rotTarget[1], rotTarget[2]);
	
	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			IP->CurrentPositionVector->VecData[i] = frame.p(i); 
		}
		else {
			IP->CurrentPositionVector->VecData[i] = rot[i-3];
		}
		
		IP->CurrentVelocityVector->VecData[i] = 0; //aus KDL
		IP->CurrentAccelerationVector->VecData[i] = 0; // aus KDL

		IP->MaxVelocityVector->VecData[i] = 3; // aus TASK oder BSP
		IP->MaxAccelerationVector->VecData[i] = 0.01; //aus TASK oder BSP
		IP->MaxJerkVector->VecData[i] = 1.0;	//aus TASK oder BSP

		if (i < 3) {
			IP->TargetPositionVector->VecData[i] = target.p(i); 
		}
		else {
			IP->TargetPositionVector->VecData[i] = rotTarget[i-3];
		}
		
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

int RobotKinematicLINGen::nextCycle(KDL::JntArrayAcc * nextVector)
{
	int ret = RML->RMLPosition(*IP, OP, *Flags);

	*IP->CurrentPositionVector = *OP->NewPositionVector;
	*IP->CurrentVelocityVector = *OP->NewVelocityVector;
	*IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
	
	/*
	printf("-----------\n");
	for (int i = 0; i < 6; i++) {
		printf("%f %f %f\n\n", OP->NewPositionVector->VecData[i], OP->NewVelocityVector->VecData[i], OP->NewAccelerationVector->VecData[i]);
	}
	printf("\n");
	*/
	
	KDL::Rotation rot = KDL::Rotation::RPY(OP->NewPositionVector->VecData[3], OP->NewPositionVector->VecData[4], OP->NewPositionVector->VecData[5]);
	KDL::Vector vec(OP->NewPositionVector->VecData[0], OP->NewPositionVector->VecData[1], OP->NewPositionVector->VecData[2]);
	KDL::Frame newFrame(rot,vec);

	KDL::JntArray guess = nextVector->value();

	this->kinematic->IK(&newFrame, &guess, &nextVector->q);


	return ret;
}
