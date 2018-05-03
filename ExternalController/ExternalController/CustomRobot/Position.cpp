#include "Position.h"

Position::Position(KDL::Frame frame)
{
	this->type = PositionType::FRAME;
	this->frame = frame;
}

Position::Position(KDL::JntArray joints)
{
	this->type = PositionType::CONFIGURATION;
	this->configuration = joints;
}

Position::~Position()
{

}

KDL::JntArray Position::getConfig(RobotKinematicManager *kin, const KDL::JntArray *guess)
{
	if (this->type == PositionType::FRAME) {
		if (kin == NULL) {
			throw - 1;
		}
		else {

			KDL::JntArray newArray(kin->getDOF());
			kin->IK(&this->frame, guess, &newArray);

			return newArray;
		}
	}

	return this->configuration;
}

KDL::Frame Position::getFrame(RobotKinematicManager *kin)
{
	if (this->type == PositionType::CONFIGURATION) {
		if (kin == NULL) {
			throw - 1;
		}
		else {
			KDL::Frame newFrame;
			kin->FK(&this->configuration, &newFrame);
			return newFrame;
		}
	}

	return this->frame;
}
