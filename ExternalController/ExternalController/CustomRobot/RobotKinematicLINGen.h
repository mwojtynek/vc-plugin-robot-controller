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

namespace CSEController {
	class RobotKinematicLINGen
	{
	private:

		ReflexxesAPI * RML;
		RMLPositionInputParameters  *IP;
		RMLPositionOutputParameters *OP;
		RMLPositionFlags            *Flags;

		RobotKinematicManager * kinematic;

	public:
		RobotKinematicLINGen(RobotKinematicManager * kinematic, double cycleTime);
		~RobotKinematicLINGen();

		int init(KDL::JntArrayAcc *startState, Task *task);
		int nextCycle(KDL::JntArrayAcc *nextState);
	};
}