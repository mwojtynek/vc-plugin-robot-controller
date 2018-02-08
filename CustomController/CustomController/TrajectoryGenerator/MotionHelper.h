#pragma once

#include<vector>

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
	int len;
	std::vector<double> *_times;
	std::vector<double> _jerks;
	Kin _start;
	//Nur zur Beschreibung. Wird nie verwendet
	Kin _end;
	Kin _bound;

public:
	Motion(Kin start, Kin end, Kin bound, std::vector<double> * times);

	void setJerk(int segment, double newJerk);
	void setJerk(std::vector<double> newJerks);

	double getValue(double time, KinType typebool);
	Kin getValue(double time);

	Kin getStart();
	Kin getEnd();
	Kin getBound();
	int getPhaseCount();
};