#pragma once

#include <memory>
#include <cfloat>
#include "data_structure.h"
#include "reference_line.h"

namespace PathPlanning
{

struct BoundaryPoint {
    double s = 0.0;
    double lb_l = DBL_MIN;
    double ub_l = DBL_MAX;
    XYPosition lb_xy;
    XYPosition ub_xy;
};
    
class FreeSpace {
public:
    FreeSpace() = default;
    void set_reference_line(std::shared_ptr<ReferenceLine> ref_ptr) { _p_reference_line = ref_ptr; }
    std::shared_ptr<const ReferenceLine> reference_line_ptr() const { return _p_reference_line; }
    const std::vector<BoundaryPoint>& boundary_points() const { return _boundary_points; }
    std::vector<BoundaryPoint>* mutable_boundary_points() { return &_boundary_points; }
    bool get_l_bound_for_circle(double s, double r, double* lower_bound, double* upper_bound) const;
    void update_circle_bounds(double r);
    BoundaryPoint get_circle_bound(double s) const;
private:
    std::shared_ptr<ReferenceLine> _p_reference_line;
    std::vector<BoundaryPoint> _boundary_points;
    std::vector<BoundaryPoint> _circle_bounds;
};

} // namespace PathPlanning
