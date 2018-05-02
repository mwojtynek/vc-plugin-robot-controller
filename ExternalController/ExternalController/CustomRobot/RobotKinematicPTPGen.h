#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "Task.h"

#include <kdl\jntarrayacc.hpp>


class RobotKinematicPTPGen
{
private:

	int DOF;

	ReflexxesAPI                *RML;
	RMLPositionInputParameters  *IP;
	RMLPositionOutputParameters *OP;
	RMLPositionFlags            *Flags;

public:
	RobotKinematicPTPGen(int DOF, double cycleTime);
	~RobotKinematicPTPGen();

	int init(KDL::JntArrayAcc *startState, Task *task);
	int nextCycle(KDL::JntArrayAcc *nextState);
};

