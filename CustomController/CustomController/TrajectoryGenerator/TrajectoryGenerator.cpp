// TrajectoryGenerator.cpp: Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <math.h>
#include <nlopt.hpp>
#include <stdio.h>
#include <vector>

#include <chrono>

#include "MotionHelper.h"
#include "OptimizationFunctions.h"

void PlotMotion(const char *name, Motion *motion, double *times) {

	#pragma warning(suppress : 4996)
	FILE *file = fopen(name, "w");

	double t_cumulative[8];

	for (int i = 0; i < 8; i++) {
		if (i == 0) {
			t_cumulative[0] = 0;
			continue;
		}
		t_cumulative[i] = t_cumulative[i - 1] + times[i-1];
	}
	
	fprintf(file, "T j a v s\n");
	for (double dt = -0.5; dt < t_cumulative[7] + 0.5; dt += 0.001) {
		Kin k;
		if (dt < 0) {
			k = motion->getStart();
		}
		else {
			k = motion->getValue(times, dt);
		}
				
		fprintf(file, "%f %f %f %f %f\n",
			dt,
			k.j,
			k.a,
			k.v,
			k.s
		);
	}
	fclose(file);
}

int main()
{
	Kin bound;
	bound.j = 2.0;
	bound.a = 1.0;
	bound.v = 1.0;
	//s hat keine Grenzen
	
	Kin init1, init2;

	//j ist keine Zustandsvariable
	init1.a = 0.0;
	init1.v = 0.0;
	init1.s = 0.0;

	init2.a = 0.5;
	init2.v = 0.5;
	init2.s = 2.0;

	Kin fin1, fin2;

	//j ist immernoch keine Zustandsvariable
	fin1.a = 0.0;
	fin1.v = 0.0;
	fin1.s = 2.0;

	fin2.a = 0.0;
	fin2.v = 0.1;
	fin2.s = 5.0;
	

	Motion mot1(init1, fin1, bound, 0);
	Motion mot2(init2, fin2, bound, 1);

	std::vector<Motion *> motions;
	motions.push_back(&mot1); 
	motions.push_back(&mot2);

	int size = TIME_SAMPLES + JERK_DATA * motions.size();
	double *states = new double[size];
	
	// Loesen mit Laufzeitberechnung
	auto start = std::chrono::high_resolution_clock::now();
		solve(&motions, states, size);
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	printf("Needed %f milliseconds\n", elapsed.count() * 1000);

	for (int i = 0; i < size; i++) {
		printf("%f\n", states[i]);
	}

	// Make Plot
	for (int i = 0; i < motions.size(); i++) {
		char filename[100];
		#pragma warning(suppress : 4996)
		sprintf(filename, "trajectory/path%d.txt", i+1);
		PlotMotion(filename, motions[i], states);
	}
	system("python visualize.py");
	delete [] states;
}

