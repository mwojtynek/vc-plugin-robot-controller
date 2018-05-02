#include "KinematicVector.h"

#include <stdio.h>
#include <string.h>

KinematicVector::KinematicVector(const KinematicVector & cpy)
{
	
	this->DOF = cpy.DOF;
	int memsize = this->DOF * sizeof(double);

	this->pos = new double[this->DOF];
	this->vel = new double[this->DOF];
	this->acc = new double[this->DOF];

	memcpy(this->pos, cpy.pos, memsize);
	memcpy(this->vel, cpy.vel, memsize);
	memcpy(this->acc, cpy.acc, memsize);

	
}

KinematicVector & KinematicVector::operator=(KinematicVector cpy)
{
	if (this->DOF != cpy.DOF) {
		if (this->DOF != 0) {
			delete[] this->pos;
			delete[] this->vel;
			delete[] this->acc;
		}

		this->DOF = cpy.DOF;

		this->pos = new double[this->DOF];
		this->vel = new double[this->DOF];
		this->acc = new double[this->DOF];
	}
	int memsize = this->DOF * sizeof(double);
	memcpy(this->pos, cpy.pos, memsize);
	memcpy(this->vel, cpy.vel, memsize);
	memcpy(this->acc, cpy.acc, memsize);
	
	return (*this);
}

KinematicVector::KinematicVector()
{
	this->DOF = 0;
	this->pos = NULL;
	this->vel = NULL;
	this->acc = NULL;
}

KinematicVector::KinematicVector(int DOF, double * pos, double * vel, double * acc)
{
	this->DOF = DOF;

	this->pos = new double[DOF];
	this->vel = new double[DOF];
	this->acc = new double[DOF];

	int MEMsize = DOF * sizeof(double);

	if (pos == NULL) {
		memset(this->pos, 0, MEMsize);
	}
	else {
		memcpy(this->pos, pos, MEMsize);
	}
	
	if (vel == NULL) {
		memset(this->vel, 0, MEMsize);
	}
	else {
		memcpy(this->vel, vel, MEMsize);
	}


	if (acc == NULL) {
		memset(this->acc, 0, MEMsize);
	}
	else {
		memcpy(this->acc, acc, MEMsize);
	}
}

KinematicVector::~KinematicVector()
{
	if (DOF == 0) return;
	delete[] this->pos;
	delete[] this->vel;
	delete[] this->acc;
}

void KinematicVector::getPos(double * pos)
{
	memcpy(pos, this->pos, DOF * sizeof(double));
}

void KinematicVector::getVel(double * vel)
{
	memcpy(vel, this->vel, DOF * sizeof(double));
}

void KinematicVector::getAcc(double * acc)
{
	memcpy(acc, this->acc, DOF * sizeof(double));
}

double KinematicVector::getPos(unsigned int index)
{
	if (index >= DOF) {
		throw - 1;
	}
	return this->pos[index];
}

double KinematicVector::getVel(unsigned int index)
{
	if (index >= DOF) {
		throw - 1;
	}
	return this->vel[index];
}

double KinematicVector::getAcc(unsigned int index)
{
	if (index >= DOF) {
		throw - 1;
	}
	return this->acc[index];
}
