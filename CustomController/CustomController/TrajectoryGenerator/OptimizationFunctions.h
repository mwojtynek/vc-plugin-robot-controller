#pragma once

#include "MotionHelper.h"
#include <vector>
#include <nlopt.hpp>

nlopt::result solve(Motion &motion, std::vector<double> &x);

