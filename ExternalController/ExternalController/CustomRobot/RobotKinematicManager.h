#pragma once

#include <string>
#include <vector>

#include <kdl\chain.hpp>
#include <kdl\jntarray.hpp>

#include <kdl\chainfksolverpos_recursive.hpp>
#include <kdl\chainiksolverpos_lma.hpp>


/*
	TODO:
		Kinetic Limits!
		Getter for them
		organization of the joints
*/

// Orocos-KDL Binding
namespace CSEController {
	class RobotKinematicManager
	{

	private:
		//solvers, pre-initialized
		KDL::ChainFkSolverPos_recursive *fkpos_solver;
		KDL::ChainIkSolverPos_LMA *ikpos_solver;

		KDL::JntArray defaultGuess;

	public:
		KDL::Chain chain;

		int getDOF();

		RobotKinematicManager(std::string urdf_path);

		~RobotKinematicManager();

		int FK(const KDL::JntArray *joints, KDL::Frame *frame);
		//int SpeedFK(const double * joints, const double * jointsDot, double * twist);

		int IK(const KDL::Frame *frame, const KDL::JntArray * guess, KDL::JntArray *joints);
	};

}