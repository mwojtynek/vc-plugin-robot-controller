#pragma once


#include <frames_io.hpp>

double d2r(double deg);
// double r2d(double rad);

// Variablen vorher alloziieren!
// dFrame 12 Elemente gro�!
// dVector 3 Elemente gro�!
// dJoints n Elemente gro�! n ist dabei Anzahl von joints; sollte man selber wissen wie viele Joints JntArray hat
// dTwist  6 Elemente gro�!

// bei jntToDouble und doubleToJnt kann NULL f�r die Konvertierungsfunktion eingegeben werden. Konvertierung ist insbesondere f�r d2r oder r2d gedacht.

// To Doubles
void frameToDouble(double * dFrame, KDL::Frame frame); 
void vectorToDouble(double * dVector, KDL::Vector vector);	
void jntToDouble(double * dJoints, KDL::JntArray joints, double (*conv)(double) = NULL );
void twistToDouble(double * dTwist, KDL::Twist twist);

// To KDL
void doubleToFrame(KDL::Frame  &frame, const double * dFrame);
void doubleToVector(KDL::Vector &vector, const double * dVector);
void doubleToJnt(KDL::JntArray &joints, const double * dJoints, double(*conv)(double) = NULL); // Da die Anzahl an Joints nicht aus JntArray entnommen werden kann, muss die �bergeben werden!
void doubleToTwist(KDL::Twist &twist, const double * dTwist);