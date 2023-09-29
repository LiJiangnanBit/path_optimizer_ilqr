#include "reference_line.h"
#include "tool.h"
#include <float.h>

namespace PathPlanning {

void ReferenceLine::initialize(const tk::spline& x_s, const tk::spline& y_s, double length) {
    _x_s = x_s;
    _y_s = y_s;
    _length = length;
    _is_initialized = true;
}

PathPoint ReferenceLine::get_reference_point(double s) const {
    PathPoint ret;
    ret.x = _x_s(s);
    ret.y = _y_s(s);
    double x_d1 = _x_s.deriv(1, s);
    double y_d1 = _y_s.deriv(1, s);
    ret.theta = atan2(y_d1, x_d1);
    double x_d2 = _x_s.deriv(2, s);
    double y_d2 = _y_s.deriv(2, s);
    ret.kappa = (x_d1 * y_d2 - y_d1 * x_d2) / pow(pow(x_d1, 2) + pow(y_d1, 2), 1.5);
    return ret;
}

XYPosition ReferenceLine::get_xy_by_sl(const SLPosition& sl) const {
    const auto ref_pt = get_reference_point(sl.s);
    double norm = constrainAngle(ref_pt.theta + M_PI_2);
    return {ref_pt.x + sl.l * cos(norm), ref_pt.y + sl.l * sin(norm)};
}

SLPosition ReferenceLine::get_projection(const XYPosition& xy) const {
    const double grid = 1.0;
    double tmp_s = 0.0, min_dis_s = 0.0;
    auto min_dis = DBL_MAX;
    while (tmp_s <= _length) {
        XYPosition ref_pt{_x_s(tmp_s), _y_s(tmp_s)};
        double tmp_dis = distance(ref_pt, xy);
        if (tmp_dis < min_dis) {
            min_dis = tmp_dis;
            min_dis_s = tmp_s;
        }
        tmp_s += grid;
    }
    // Newton's method
    return get_projection_by_newton(xy, tmp_s);
}

SLPosition ReferenceLine::get_projection_by_newton(const XYPosition& xy, double hint_s) const {
    hint_s = std::min(hint_s, _length);
    double cur_s = hint_s;
    double prev_s = hint_s;
    double x = 0.0, y = 0.0;
    for (int i = 0; i < 20; ++i) {
        x = _x_s(cur_s);
        y = _y_s(cur_s);
        double dx = _x_s.deriv(1, cur_s);
        double dy = _y_s.deriv(1, cur_s);
        double ddx = _x_s.deriv(2, cur_s);
        double ddy = _y_s.deriv(2, cur_s);
        // Ignore coeff 2 in J and H.
        double j = (x - xy.x) * dx + (y - xy.y) * dy;
        double h = dx * dx + (x - xy.x) * ddx + dy * dy + (y - xy.y) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - prev_s) < 1e-5) break;
        prev_s = cur_s;
    }

    cur_s = std::min(cur_s, _length);
    return SLPosition{cur_s, distance(xy, XYPosition{x, y})};
}

}