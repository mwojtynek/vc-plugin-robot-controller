#include "RobotKinematicManager.h"

#include <kdl\frames_io.hpp>
#include <kdl\tree.hpp>


#include <kdl_parser.hpp>
#include <stdio.h>

using namespace KDL;

int RobotKinematicManager::getDOF()
{
	return this->chain.getNrOfJoints();
}

RobotKinematicManager::RobotKinematicManager(std::string urdf_path)
{

	std::cout << urdf_path << std::endl;
	KDL::Tree tree;
	
	if (!kdl_parser::treeFromFile(urdf_path, tree)) {
		printf("Something went wrong with reading the urdf file.\n");
		return;
	}
	if (!tree.getChain("base_link", "tool0", this->chain)) {
		printf("Something went wrong with chain.\n");
	}

	fkpos_solver = new ChainFkSolverPos_recursive(this->chain);
	
	// macht, dass das +/- immerhin funktioniert. genaue Funktion immernoch nicht verstanden aber es laeuft; so who cares? \_(^,^)_/
	Eigen::Matrix< double, 6, 1 > mat;
	mat << 1, 1, 0, 0, 0, 1;

	ikpos_solver = new ChainIkSolverPos_LMA(this->chain, mat);
	defaultGuess = JntArray(this->chain.getNrOfJoints());

}

RobotKinematicManager::~RobotKinematicManager()
{
	delete fkpos_solver;
	delete ikpos_solver;
}

int RobotKinematicManager::FK(const KDL::JntArray *joints, KDL::Frame *frame)
{
	
	if (joints->rows() != chain.getNrOfJoints()) {
		throw - 1; //TODO
	}
	
	return this->fkpos_solver->JntToCart(*joints, *frame);
	
}

int RobotKinematicManager::IK(const KDL::Frame * frame, const KDL::JntArray * guess, KDL::JntArray * joints)
{

	const KDL::JntArray *guess_actual;
	if (guess == NULL) {
		guess_actual = &this->defaultGuess;
	}
	else {
		guess_actual = guess;
	}

	return this->ikpos_solver->CartToJnt(*guess_actual, *frame, *joints);
	
}

/*
int RobotKinematicManager::SpeedFK(const double * joints, const double * jointsDot, double * twist)
{
	ChainFkSolverVel_recursive fksolver = ChainFkSolverVel_recursive(chain);
	int jntCount = chain.getNrOfJoints();
	JntArray q(jntCount);
	JntArray qdot(jntCount);
	Marshal::doubleToJnt(q, joints, d2r);
	Marshal::doubleToJnt(qdot, jointsDot, d2r);
	JntArrayVel qVel(q, qdot);
	FrameVel frameVel;
	if (fksolver.JntToCart(qVel, frameVel) < 0) {
		return -1;
	}
	Marshal::vectorToDouble(twist, frameVel.p.v);
	Marshal::vectorToDouble(twist + 3, frameVel.M.w);
	return 0;
}*/

