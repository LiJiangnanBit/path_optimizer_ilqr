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

PathPoint local_to_global(const PathPoint& ref, const PathPoint& target) {
    PathPoint ret = target;
    ret.x = target.x * cos(ref.theta) - target.y * sin(ref.theta) + ref.x;
    ret.y = target.x * sin(ref.theta) + target.y * cos(ref.theta) + ref.y;
    ret.theta = ref.theta + target.theta;
    return ret;
}

PathPoint global_to_local(const PathPoint& ref, const PathPoint& target) {

}

}