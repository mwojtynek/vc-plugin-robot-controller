#pragma once

#include "MotionHelper.h"
#include <vector>
#include <nlopt.h>

nlopt_result solve(std::vector<Motion *> *motion, double *x, int size);

