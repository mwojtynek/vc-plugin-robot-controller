#pragma once

#include<vector>

#define TIME_SAMPLES 7  // Halt fuer die 7 Phasen der Bewegung
#define JERK_DATA	 4	// Die vier Phasen mit Jerk

typedef enum {
	Jtype = 0,
	Atype = 1,
	Vtype = 2,
	Stype = 3
} KinType;

typedef struct {
	double j;
	double a;
	double v;
	double s;
} Kin;

double absoluteTime(const std::vector<double> &times, int k);
double absoluteTime(const double *times, int k);

class Motion {
private:
	int _id;
		
	Kin _start;
	Kin _end;
	Kin _bound;

public:
	Motion(Kin start, Kin end, Kin bound, int id);
	
	double getValue(const double * states, double time, KinType typebool);
	Kin getValue(const double * states, double time);

	Kin getStart();
	Kin getEnd();
	Kin getBound();
};