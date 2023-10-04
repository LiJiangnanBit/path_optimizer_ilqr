#include "tool.h"

namespace PathPlanning {

double distance(const XYPosition& p1, const XYPosition& p2) {
    return std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2));
}

double constrainAngle(double angle) {
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