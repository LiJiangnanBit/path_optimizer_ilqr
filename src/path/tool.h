#pragma once
#include <cmath>
#include "data_structure.h"

namespace PathPlanning {

double distance(const XYPosition& p1, const XYPosition& p2) {
    return std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2));
}

template<typename T>
T constrainAngle(T angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
        return constrainAngle(angle);
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
        return constrainAngle(angle);
    } else {
        return angle;
    }
}

}