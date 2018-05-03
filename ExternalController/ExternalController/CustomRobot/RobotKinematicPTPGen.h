#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "Task.h"
#include "RobotKinematicManager.h"

#include <kdl\jntarrayacc.hpp>


class RobotKinematicPTPGen
{
private:

	RobotKinematicManager * kinematic;

	ReflexxesAPI                *RML;
	RMLPositionInputParameters  *IP;
	RMLPositionOutputParameters *OP;
	RMLPositionFlags            *Flags;

public:
	RobotKinematicPTPGen(RobotKinematicManager * kinematic, double cycleTime);
	~RobotKinematicPTPGen();

	int init(KDL::JntArrayAcc *startState, Task *task);
	int nextCycle(KDL::JntArrayAcc *nextState);
};

