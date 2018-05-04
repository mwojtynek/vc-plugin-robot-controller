#pragma once

#include "RobotKinematicManager.h"
#include "Task.h"

#include "RobotKinematicPTPGen.h"
#include "RobotKinematicLINGen.h"

#include <kdl\jntarrayacc.hpp>

#include <queue>

// Reflexxes Binding
namespace CSEController {
	class RobotMotionManager
	{
	private:

		RobotKinematicManager * kinematic;

		Task *currentTask;
		std::queue<Task *> taskList;


		RobotKinematicPTPGen *ptp;
		RobotKinematicLINGen *lin;

	public:
		KDL::JntArrayAcc state;

		RobotMotionManager(RobotKinematicManager *kinematic, double cycleTime);
		~RobotMotionManager();

		int init(KDL::JntArrayAcc *start, double cycleTime);
		int nextCycle(KDL::JntArrayAcc * newState);

		void addTask(Task *newTask);

	};

}