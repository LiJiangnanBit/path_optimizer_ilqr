#pragma once
#include <vector>
#include "spline.h"
#include "data_structure.h"

namespace PathPlanning {

class ReferenceLine {
public:
    ReferenceLine() : _length(0.0), _is_initialized(false) {};
    void initialize(const tk::spline& x_s, const tk::spline& y_s, double length);
    bool is_initialized() const { return _is_initialized; }
    PathPoint get_reference_point(double s) const;
    XYPosition get_xy_by_sl(const SLPosition& sl) const;
    SLPosition get_projection(const XYPosition& xy) const;
    SLPosition get_projection_by_newton(const XYPosition& xy, double hint_s) const;
    const tk::spline& x_s() const { return _x_s; }
    const tk::spline& y_s() const { return _y_s; }
    double length() const { return _length; }
private:
    tk::spline _x_s;
    tk::spline _y_s;
    double _length;
    bool _is_initialized;
};

}