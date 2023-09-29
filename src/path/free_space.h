#pragma once

#include <memory>
#include "data_structure.h"
#include "reference_line.h"

namespace PathPlanning
{

struct BoundaryPoint {
    double s = 0.0;
    double lb_l = 0.0;
    double ub_l = 0.0;
    XYPosition lb_xy;
    XYPosition ub_xy;
};
    
class FreeSpace {
public:
    FreeSpace() = default;
    void set_reference_line(std::shared_ptr<ReferenceLine> ref_ptr) { _p_reference_line = ref_ptr; }
    const std::vector<BoundaryPoint>& boundary_points() const { return _boundary_points; }
    std::vector<BoundaryPoint>* mutable_boundary_points() { return &_boundary_points; }
private:
    std::shared_ptr<ReferenceLine> _p_reference_line;
    std::vector<BoundaryPoint> _boundary_points; 
};

} // namespace PathPlanning