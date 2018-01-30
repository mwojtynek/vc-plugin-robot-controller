#pragma once

extern "C" __declspec(dllexport) int AddRobot(double *DH_Data, int jointCount, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);

extern "C" __declspec(dllexport) int FK(int id, const double * joints, double * frame);
extern "C" __declspec(dllexport) int SpeedFK(int id, const double * joints, const double * jointsDot, double * twist);