#pragma once

//KDL 
#include <chain.hpp>
#include <chainfksolver.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <frames_io.hpp>


public class CustomRobot {
private:
	//not really neccassary
	int _id;
	//KDL Parameter(DH)
	KDL::Chain chain;

	//cartesian
	double _maxSpeed;
	double _maxAcceleration;
	double _maxJerk;
	
public:
	CustomRobot(int id, double *DH_Data, int jointCoint, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);

	void setMaxSpeed(double maxSpeed);
	double getMaxSpeed();

	void setMaxAcceleration(double maxAcceleration);
	double getMaxAcceleration();

	void setMaxJerk(double maxJerk);
	double getMaxJerk();

	int FK(const double * joints, double * frame);
	int SpeedFK(const double * joints, const double * jointsDot, double * twist);
	
	int IK(const double * frame, double * joints);



};