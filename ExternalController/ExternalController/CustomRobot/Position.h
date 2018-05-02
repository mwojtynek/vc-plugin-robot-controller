#pragma once

#include <kdl\frames.hpp>

enum PositionType {FRAME, CONFIGURATION};

class Position
{
private:
	PositionType type;
	// Aufbau Frame und Configuration
	union {
		KDL::Frame frame;
		struct {
			double * joints;
			int DOF;
		} configuration;

	};
public:
	Position(KDL::Frame frame);
	Position(double * joints, int jointCount);
	~Position();

	void getConfig(double * config);
	int getDof();

	KDL::Frame getFrame();
};

