#include "OptimizationFunctions.h"
#include <nlopt.h>

#include "MotionHelper.h"

// TODO: Gradienten einbauen, bei moeglichen Verwendung von Gradienten Verfahren! LN_COBYLA braucht keine Gradienten! 
// ANM: Bei anderen Verfahren auch darauf achten, dass Equality Constraints (b_eq) erlaubt sind! LN_COBYLA ist "nur" lokal. Daher kann es (eher unwahrscheinlich) bessere Loesungen geben!

#define INEQUALITY_CONSTRAINTS_SIZE		 2	// a und v
#define   EQUALITY_CONSTRAINTS_SIZE		 3	// a v und s
#define INEQUALITY_CONSTRAINTS_TIMES     6	// 6 Zeitpunkte 
#define TOTAL_INEQUALITY_CONSTRAINTS     (INEQUALITY_CONSTRAINTS_SIZE * INEQUALITY_CONSTRAINTS_TIMES)

void b_ineq(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data) {
	
	std::vector<Motion *> *motion = (std::vector<Motion *> *)(data);
	
	for (int joint = 0; joint < motion->size(); joint++) {
		Kin bound = motion->at(joint)->getBound();

		for (int i = 0; i < INEQUALITY_CONSTRAINTS_TIMES; i++) {
			Kin k = motion->at(joint)->getValue(x, absoluteTime(x, i));
			result[joint * TOTAL_INEQUALITY_CONSTRAINTS + i * INEQUALITY_CONSTRAINTS_SIZE] = k.a * k.a - bound.a * bound.a;
			result[joint * TOTAL_INEQUALITY_CONSTRAINTS + i * INEQUALITY_CONSTRAINTS_SIZE + 1] = k.v * k.v - bound.v * bound.v;
		}
	}
		
}

void b_eq(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data) {
	
	std::vector<Motion *> *motion = (std::vector<Motion *> *)(data);
	for (int i = 0; i < motion->size(); i++) {
		Kin fin = motion->at(i)->getEnd();
		Kin k = motion->at(i)->getValue(x, absoluteTime(x, TIME_SAMPLES - 1));
		result[i * EQUALITY_CONSTRAINTS_SIZE] = k.a - fin.a;
		result[i * EQUALITY_CONSTRAINTS_SIZE + 1] = k.v - fin.v;
		result[i * EQUALITY_CONSTRAINTS_SIZE + 2] = k.s - fin.s;
	}

}

double J(unsigned n, const double *x, double *grad, void *data) {
	double sum = 0.0;
	// alle weiteren Zustaende sind frei waehlbar und bestimmen nicht die Kostenfunktion J 
	for (int i = 0; i < TIME_SAMPLES; i++) {
		sum += x[i];
	}
	return  sum;
}

nlopt_result solve(std::vector<Motion *> *motions, double *x, int size)
{
	int jointCnt = motions->size(); 

	nlopt_opt opt;
	opt = nlopt_create(NLOPT_LN_COBYLA, size);

	double *lb = new double[size];
	double *ub = new double[size];
	for (int i = 0; i < TIME_SAMPLES; i++) {
		lb[i] = 0.0;
		ub[i] = 1000.0;

		x[i] = 0.5;
	}
	for (int i = 0; i < jointCnt; i++) {
		double jmax = motions->at(i)->getBound().j;
		for (int j = 0; j < JERK_DATA; j++) {
			x[TIME_SAMPLES + i * JERK_DATA + j] = 0.5;
			lb[TIME_SAMPLES + i * JERK_DATA + j] = -jmax;
			ub[TIME_SAMPLES + i * JERK_DATA + j] = jmax;
		}
	}

	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);

	nlopt_set_min_objective(opt, J, NULL);

	int eq_size = EQUALITY_CONSTRAINTS_SIZE * jointCnt;
	int ineq_size = TOTAL_INEQUALITY_CONSTRAINTS * jointCnt;

	double *eq_tol = new double[eq_size];
	double *ineq_tol = new double[ineq_size];

	for (int i = 0; i < eq_size; i++) {
		eq_tol[i] = 1e-8;
	}
	for (int i = 0; i < ineq_size; i++) {
		ineq_tol[i] = 1e-8;
	}

	nlopt_add_inequality_mconstraint(opt, ineq_size, b_ineq, motions, ineq_tol);
	nlopt_add_equality_mconstraint(opt, eq_size, b_eq, motions, eq_tol);
	
	nlopt_set_xtol_rel(opt, 1e-4);
	//nlopt_set_maxtime(opt, 0.001);

	double minf;
	
	nlopt_result result = nlopt_optimize(opt, x, &minf);

	delete[] lb;
	delete[] ub;
	delete[] eq_tol;
	delete[] ineq_tol;
	return result;
}
