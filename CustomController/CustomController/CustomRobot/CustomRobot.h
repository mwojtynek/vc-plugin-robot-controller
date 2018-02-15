#pragma once

//KDL 
#include <chain.hpp>
#include <chainfksolver.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <frames_io.hpp>

#include <MotionHelper.h>
#include <OptimizationFunctions.h>

public class CustomRobot {
private:
	//not really neccassary
	int _id;
	//KDL Parameter(DH)
	KDL::Chain chain;

	std::vector<Kin> bounds;

	double maxCartesianSpeed;
	
public:
	CustomRobot(int id, double *DH_Data, int jointCoint, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);

	void setMaxSpeed(double maxSpeed, int joint = -1);
	double getMaxSpeed(int joint);

	void setMaxAcceleration(double maxAcceleration, int joint = -1);
	double getMaxAcceleration(int joint);

	void setMaxJerk(double maxJerk, int joint = -1);
	double getMaxJerk(int joint);

	int FK(const double * joints, double * frame);
	int SpeedFK(const double * joints, const double * jointsDot, double * twist);
		
	int IK(const double * frame, double * joints);





};