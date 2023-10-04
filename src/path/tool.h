#pragma once
#include <cmath>
#include "data_structure.h"

namespace PathPlanning {

double distance(const XYPosition& p1, const XYPosition& p2);

double constrainAngle(double angle);

}