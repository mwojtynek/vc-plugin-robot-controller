#include "StaticWrapper.h"
#include "CustomRobot.h"

#include <unordered_map>

std::unordered_map<int, CustomRobot *> dictionary;

int AddRobot(double * DH_Data, int jointCount, double maxSpeed, double maxAcceleration, double maxJerk)
{
	int id;
	int size = dictionary.size();
	
	if (dictionary.count(size) != 0) {
		for (int i = 0; i < dictionary.size(); i++) {
			if (dictionary.count(i) == 0) {
				id = i;
				break;
			}
		}
	}
	else { 
		id = size;
	}
	CustomRobot *newRobot = new CustomRobot(id, DH_Data, jointCount, maxSpeed, maxAcceleration, maxJerk);
	std::pair<int, CustomRobot *> newEntry(id, newRobot);
	dictionary.insert(newEntry);
	return id;
}

int FK(int id, const double * joints, double * frame)
{
	std::unordered_map<int, CustomRobot *>::const_iterator robotEntry = dictionary.find(id);
	if (robotEntry == dictionary.end()) {
		return -42;
	}

	CustomRobot *robot = robotEntry->second;
	return robot->FK(joints, frame);
}

int SpeedFK(int id, const double * joints, const double * jointsDot, double * twist)
{
	std::unordered_map<int, CustomRobot *>::const_iterator robotEntry = dictionary.find(id);
	if (robotEntry == dictionary.end()) {
		return -42;
	}

	CustomRobot *robot = robotEntry->second;
	return robot->SpeedFK(joints, jointsDot, twist);
}
