#include "OptimizationFunctions.h"
#include <nlopt.hpp>

#include "MotionHelper.h"

void b_ineq(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data) {
	Motion *motion = reinterpret_cast<Motion *>(data);
	// TODO gradient!
	/*
	if (!grad.empty()) {

	}*/

	Kin bound = motion->getBound();
	for (int i = 0; i < (m / 2); i++) {
		Kin k = motion->getValue(absoluteTime(x, i));
		result[i * 2] = k.a * k.a - bound.a * bound.a;
		result[i * 2 + 1] = k.v * k.v - bound.v * bound.v;
	}

}

void b_eq(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data) {
	Motion *motion = reinterpret_cast<Motion *>(data);
	// TODO gradient!
	/*
	if (!grad.empty()) {

	}*/

	Kin fin = motion->getEnd();
	Kin k = motion->getValue(absoluteTime(x, n - 1));
	result[0] = k.a - fin.a;
	result[1] = k.v - fin.v;
	result[2] = k.s - fin.s;
}

double J(const std::vector<double> &x, std::vector<double> &grad, void *data) {
	// TODO gradient!
	/*
	if (!grad.empty()) {

	}
	*/

	double sum = 0.0;
	for (int i = 0; i < x.size(); i++) {
		sum += x[i];
	}
	return  sum;
}

nlopt::result solve(Motion &motion, std::vector<double> &x)
{
	int size = motion.getPhaseCount();

	nlopt::opt opt(nlopt::LN_COBYLA, size);

	opt.set_lower_bounds(0.0);
	opt.set_upper_bounds(1000.0);

	opt.set_min_objective(J, NULL);

	std::vector<double> ineqtol(2 * (size - 1), 1e-8);
	std::vector<double> eqtol(3, 1e-8);
	opt.add_inequality_mconstraint(b_ineq, &motion, ineqtol);
	opt.add_equality_mconstraint(b_eq, &motion, eqtol);

	opt.set_xtol_rel(1e-4);

	for (int i = 0; i < size; i++) {
		x[i] = 0.5;
	}

	double minf;
	nlopt::result result = opt.optimize(x, minf);
	return result;
}
