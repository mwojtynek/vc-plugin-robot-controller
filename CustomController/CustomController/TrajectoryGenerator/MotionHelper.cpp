#include "MotionHelper.h"

#define JERK(segment)	((segment%2?0.0:states[TIME_SAMPLES + JERK_DATA * _id + segment / 2]))

double absoluteTime(const std::vector<double> &times, int k) {
	double sum = 0.0;
	for (int i = 0; i < k + 1; i++) {
		sum += times[i];
	}
	return sum;
}

double absoluteTime(const double *times, int k) {
	double sum = 0.0;
	for (int i = 0; i < k + 1; i++) {
		sum += times[i];
	}
	return sum;
}

Motion::Motion(Kin start, Kin end, Kin bound, int id)
{
	_id = id;

	_start = start;
	_end = end;
	_bound = bound;
}

double Motion::getValue(const double * states, double time, KinType typebool)
{
	Kin current = getValue(states, time);
	switch (typebool) {
	case Jtype: return current.j;
	case Atype: return current.a;
	case Vtype: return current.v;
	case Stype: return current.s;
	default: throw - 1;
	}
}

Kin Motion::getValue(const double * states, double time)
{
	int segment;
	
	for (segment = 0; segment < TIME_SAMPLES; segment++) {

		if (time <= states[segment]) break;
		time -= states[segment];
	}

	Kin current = _start;

	double dt = 0.0;
	for (int i = 0; i < segment; i++) {
		dt = states[i];
		current.s += ((JERK(i) / 6 * dt + current.a / 2) * dt + current.v) * dt;
		current.v += (JERK(i) / 2 * dt + current.a) * dt;
		current.a += JERK(i) * dt;

	}

	dt = time;
	current.j = JERK(segment);
	current.s += ((current.j / 6 * dt + current.a / 2) * dt + current.v) * dt;
	current.v += (current.j / 2 * dt + current.a) * dt;
	current.a += current.j * dt;

	return current;
}

Kin Motion::getStart()
{
	return _start;
}

Kin Motion::getEnd()
{
	return _end;
}

Kin Motion::getBound()
{
	return _bound;
}