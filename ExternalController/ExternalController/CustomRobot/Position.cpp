#include "Position.h"

#include <string.h>


Position::Position(KDL::Frame frame)
{
	type = PositionType::FRAME;
	this->frame = frame;
}

Position::Position(double * joints, int jointCount)
{
	type = PositionType::CONFIGURATION;
	this->configuration.DOF = jointCount;
	this->configuration.joints = new double[jointCount];
	memcpy(this->configuration.joints, joints, jointCount * sizeof(double));
}

Position::~Position()
{
	if (type == PositionType::CONFIGURATION) {
		delete[] this->configuration.joints;
	}
}

void Position::getConfig(double * config)
{
	if (type == PositionType::FRAME) {
		throw - 1;
	}

	memcpy(config, this->configuration.joints, this->configuration.DOF * sizeof(double));
}

int Position::getDof()
{
	if (type == PositionType::FRAME) {
		throw - 1;
	}
	return this->configuration.DOF;
}

KDL::Frame Position::getFrame()
{
	if (PositionType::CONFIGURATION) {
		throw - 1;
	}
	return this->frame;
}
