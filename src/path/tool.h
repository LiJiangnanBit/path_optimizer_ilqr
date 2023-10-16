#pragma once
#include <cmath>
#include "data_structure.h"

namespace PathPlanning {

double distance(const XYPosition& p1, const XYPosition& p2);

double constrainAngle(double angle);

PathPoint local_to_global(const PathPoint& ref, const PathPoint& target);

PathPoint global_to_local(const PathPoint& ref, const PathPoint& target);

}