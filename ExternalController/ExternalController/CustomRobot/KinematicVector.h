#pragma once
#include <stdlib.h>

class KinematicVector
{
public:
	KinematicVector(const KinematicVector &cpy);
	KinematicVector & operator= (KinematicVector cpy);
	
	KinematicVector();
	KinematicVector(int DOF, double * pos = NULL, double * vel = NULL, double * acc = NULL);
	~KinematicVector();

	void getPos(double * pos);
	void getVel(double * vel);
	void getAcc(double * acc);

	double getPos(unsigned int index);
	double getVel(unsigned int index);
	double getAcc(unsigned int index);

	double * pos;
	double * vel;
	double * acc;

	int DOF;
};

