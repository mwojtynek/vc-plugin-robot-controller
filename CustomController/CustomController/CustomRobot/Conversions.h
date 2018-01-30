#pragma once


#include <frames_io.hpp>

double d2r(double deg);
// double r2d(double rad);

// Variablen vorher alloziieren!
// dFrame 12 Elemente groß!
// dVector 3 Elemente groß!
// dJoints n Elemente groß! n ist dabei Anzahl von joints; sollte man selber wissen wie viele Joints JntArray hat
// dTwist  6 Elemente groß!

// bei jntToDouble und doubleToJnt kann NULL für die Konvertierungsfunktion eingegeben werden. Konvertierung ist insbesondere für d2r oder r2d gedacht.

// To Doubles
void frameToDouble(double * dFrame, KDL::Frame frame); 
void vectorToDouble(double * dVector, KDL::Vector vector);	
void jntToDouble(double * dJoints, KDL::JntArray joints, double (*conv)(double) = NULL );
void twistToDouble(double * dTwist, KDL::Twist twist);

// To KDL
void doubleToFrame(KDL::Frame  &frame, const double * dFrame);
void doubleToVector(KDL::Vector &vector, const double * dVector);
void doubleToJnt(KDL::JntArray &joints, const double * dJoints, double(*conv)(double) = NULL); // Da die Anzahl an Joints nicht aus JntArray entnommen werden kann, muss die übergeben werden!
void doubleToTwist(KDL::Twist &twist, const double * dTwist);