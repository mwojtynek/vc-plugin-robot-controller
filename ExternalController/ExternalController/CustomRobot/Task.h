#pragma once

#include "Position.h"

namespace CSEController {

	enum TaskType {
		PTP,
		LIN
	};

	typedef struct {
		TaskType type;
		Position *target;

		double vConstraint;
		//PTP 0-1 als % // default 100%
		//LIN v in mm/s // immer fraglich ob das ueberhaupt eingetragen werden kann.... daher default 400 mm/s angenommen
		//double aConstraint; // erstmal weglassen TODO
		//PTP 0-1 als %
		//LIN a in mm/s^2
	} Task;

}