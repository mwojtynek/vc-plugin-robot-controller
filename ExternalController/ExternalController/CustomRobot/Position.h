#pragma once

#include <kdl\frames.hpp>
#include <kdl\jntarray.hpp>

#include "RobotKinematicManager.h"


namespace CSEController {
	enum PositionType { FRAME, CONFIGURATION };

	class Position
	{
	private:
		PositionType type;
		// Aufbau Frame und Configuration
		union {
			KDL::Frame frame;
			KDL::JntArray configuration;
		};
	public:
		Position(KDL::Frame frame);
		Position(KDL::JntArray joints);
		~Position();

		KDL::JntArray getConfig(RobotKinematicManager *kin = NULL, const KDL::JntArray *guess = NULL);
		KDL::Frame getFrame(RobotKinematicManager *kin = NULL);
	};

}