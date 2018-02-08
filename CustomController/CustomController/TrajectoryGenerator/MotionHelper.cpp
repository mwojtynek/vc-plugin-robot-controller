#include "MotionHelper.h"

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

Motion::Motion(Kin start, Kin end, Kin bound, std::vector<double> *times)
{
	len = times->size();
	_times = times;
	_jerks = std::vector<double>(len, 0);

	_start = start;
	_end = end;
	_bound = bound;
}

void Motion::setJerk(int segment, double newJerk)
{
	if (segment < 0 || segment >= _jerks.size()) {
		throw - 1;
	}
	_jerks[segment] = newJerk;
}

void Motion::setJerk(std::vector<double> newJerks)
{
	if (_jerks.size() != newJerks.size()) {
		throw - 1;
	}
	_jerks = newJerks;
}

double Motion::getValue(double time, KinType typebool)
{
	Kin current = getValue(time);
	switch (typebool) {
	case Jtype: return current.j;
	case Atype: return current.a;
	case Vtype: return current.v;
	case Stype: return current.s;
	default: throw - 1;
	}
}

Kin Motion::getValue(double time)
{
	int segment;

	for (segment = 0; segment < _times->size(); segment++) {

		if (time <= (*_times)[segment]) break;
		time -= (*_times)[segment];
	}
	Kin current = _start;

	double dt = 0.0;
	for (int i = 0; i < segment; i++) {
		dt = (*_times)[i];
		current.s += ((_jerks[i] / 6 * dt + current.a / 2) * dt + current.v) * dt;
		current.v += (_jerks[i] / 2 * dt + current.a) * dt;
		current.a += _jerks[i] * dt;

	}

	dt = time;
	current.j = _jerks[segment];
	current.s += ((_jerks[segment] / 6 * dt + current.a / 2) * dt + current.v) * dt;
	current.v += (_jerks[segment] / 2 * dt + current.a) * dt;
	current.a += _jerks[segment] * dt;

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

int Motion::getPhaseCount()
{
	return len;
}
