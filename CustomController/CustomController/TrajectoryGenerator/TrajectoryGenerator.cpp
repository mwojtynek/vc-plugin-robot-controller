// TrajectoryGenerator.cpp: Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <math.h>
#include <nlopt.hpp>
#include <stdio.h>
#include <vector>

#include <chrono>

#include "MotionHelper.h"
#include "OptimizationFunctions.h"

double jmax = 2;
double amax = 1;
double vmax = 1;

double j[7] = { jmax, 0, -jmax, 0, -jmax, 0, jmax };

double t[8];
double t_cumulative[8];
double a[8];
double v[8];
double s[8];

double a_t;
double v_t;
double s_t;

int main()
{
#pragma warning(suppress : 4996)
	FILE *file = fopen("path.txt", "w");


	a[0] = 0.0;
	v[0] = 0.0;
	s[0] = 0.0;


	Kin init, fin, bound;
	init.a = 0.7;
	init.v = 0.8;
	init.s = 0.0;

	fin.a = 0.0;
	fin.v = 0.0;
	fin.s = 3.5;

	s_t = fin.s;
	v_t = fin.v;
	a_t = fin.a;

	bound.j = 2.0;
	bound.a = 1.0;
	bound.v = 1.0;


	std::vector<double> vtimes(7, 0.0);
	Motion mot(init, fin, bound, &vtimes);

	for (int i = 0; i < 7; i++) {
		mot.setJerk(i, j[i]);
	}

	auto start = std::chrono::high_resolution_clock::now();

	//TrapZeroTrapImproved(vtimes, mot);

	//TrapZeroTrap();

	solve(mot, vtimes);

	auto finish = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> elapsed = finish - start;
	printf("Needed %f milliseconds\n", elapsed.count() * 1000);

	for (int i = 0; i < 8; i++) {
		if (i == 0) {
			t_cumulative[0] = 0;
			continue;
		}
		t_cumulative[i] = t_cumulative[i - 1] + vtimes[i - 1];
		printf("%f %f\n", vtimes[i - 1], t_cumulative[i]);

	}


	// Make Plot

	fprintf(file, "T j a v s\n");
	for (double dt = -0.5; dt < t_cumulative[7] + 0.5; dt += 0.001) {
		if (dt < 0) {
			fprintf(file, "%f %f %f %f %f\n", dt, 0.0, 0.0, 0.0, 0.0);
			continue;
		}
		if (dt > t_cumulative[7]) {
			Kin k = mot.getValue(t_cumulative[7]);
			fprintf(file, "%f %f %f %f %f\n",
				dt,
				k.j,
				k.a,
				k.v,
				k.s
			);
			continue;
		}
		Kin k = mot.getValue(dt);
		fprintf(file, "%f %f %f %f %f\n",
			dt,
			k.j,
			k.a,
			k.v,
			k.s
		);
	}
	fclose(file);
	system("python visualize.py");
}

