#include "RobotMotionManager.h"

#include "Conversions.h"

using namespace CSEController;

RobotMotionManager::RobotMotionManager(RobotKinematicManager *kinematic, double cycleTime) : state(kinematic->getDOF())
{
	this->kinematic = kinematic;
	this->ptp = new RobotKinematicPTPGen(kinematic, cycleTime);
	this->lin = new RobotKinematicLINGen(kinematic, cycleTime);
		
	currentTask = NULL;
}

RobotMotionManager::~RobotMotionManager()
{	
	delete this->lin;
	delete this->ptp;
}

int RobotMotionManager::init(KDL::JntArrayAcc *start, double cycleTime)
{
	state = *start;
	return 0;
	//TODO delete Tasks
}

int RobotMotionManager::nextCycle(KDL::JntArrayAcc * newState)
{	
	int cycleReturn = 0;
	int interpolatorReturn;
	
	if (currentTask == NULL ) {
		if (! taskList.empty()) {
			currentTask = taskList.front();
			switch (currentTask->type) {
			case PTP:
				ptp->init(&this->state, currentTask);
				break;
			case LIN:
				lin->init(&this->state, currentTask);
				break;
			}
			cycleReturn |= 2;
			taskList.pop();
		}
	}
	
	if (currentTask != NULL) {
		switch (currentTask->type) {
		case PTP:
			interpolatorReturn = ptp->nextCycle(&this->state);
			break;
		case LIN:
			interpolatorReturn = lin->nextCycle(&this->state);
			break;
		}

		if (interpolatorReturn == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
			cycleReturn |= 4;
			delete currentTask->target;
			delete currentTask;
			currentTask = NULL;

			KDL::Frame frame;
			double rpy[6];
			this->kinematic->FK(& this->state.q, &frame);
			frameToRPY(rpy, &frame);
			for (int i = 0; i < 6; i++) {
				if (i > 2) {
					printf("%f ", r2d(rpy[i]));
				}
				else {
					printf("%f ", rpy[i]);
				}
			}
			printf("finished\n");
		}
		else if (interpolatorReturn != ReflexxesAPI::RML_WORKING) {
			cycleReturn |= 1;
		}
	}
	
	*newState = this->state;

	return cycleReturn;
}

void RobotMotionManager::addTask(Task * newTask)
{
	taskList.push(newTask);
}
