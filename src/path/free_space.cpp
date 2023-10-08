#include <cmath>
#include "free_space.h"

namespace PathPlanning {
bool FreeSpace::get_l_bound_for_circle(double s, double r, double* lower_bound, double* upper_bound) const {
    *lower_bound = -20.0;
    *upper_bound = 20.0;
    if (s < 0.0 || s > _p_reference_line->length()) {
        return false;
    }
    const auto ref_pt = _p_reference_line->get_reference_point(s);
    Vector norm_vec{cos(ref_pt.theta + M_PI_2), sin(ref_pt.theta + M_PI_2)};
    auto comp = [](const BoundaryPoint& pt, double s) { return pt.s < s; };
    const auto boundary_start_iter = std::lower_bound(_boundary_points.begin(), _boundary_points.end(), s - r, comp);
    const auto boundary_end_iter = std::lower_bound(_boundary_points.begin(), _boundary_points.end(), s + r, comp);
    
    for (auto iter = boundary_start_iter; iter < boundary_end_iter; ++iter) {
        // Upper bound.
        Vector ref_to_upper_bound{iter->ub_xy.x - ref_pt.x, iter->ub_xy.y - ref_pt.y};
        const double ub_dist_to_norm = std::fabs(ref_to_upper_bound.x * norm_vec.y - ref_to_upper_bound.y * norm_vec.x);
        if (ub_dist_to_norm < r) {
            const double bound_l = ref_to_upper_bound.x * norm_vec.x + ref_to_upper_bound.y * norm_vec.y;
            *upper_bound = std::min(*upper_bound, bound_l - std::sqrt(r * r - ub_dist_to_norm * ub_dist_to_norm));
        }
        // Lower bound.
        Vector ref_to_lower_bound{iter->lb_xy.x - ref_pt.x, iter->lb_xy.y - ref_pt.y};
        const double lb_dist_to_norm = std::fabs(ref_to_lower_bound.x * norm_vec.y - ref_to_lower_bound.y * norm_vec.x);
        if (lb_dist_to_norm < r) {
            const double bound_l = ref_to_lower_bound.x * norm_vec.x + ref_to_lower_bound.y * norm_vec.y;
            *lower_bound = std::max(*lower_bound, bound_l + std::sqrt(r * r - lb_dist_to_norm * lb_dist_to_norm));
        }
    }
    return true;
}
}