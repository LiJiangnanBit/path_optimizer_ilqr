#include <tinyspline_ros/tinysplinecpp.h>
#include <glog/logging.h>
#include <cfloat>
#include "reference_line_processor.h"
#include "path/tool.h"

using namespace PathPlanning;
namespace Test {

const double max_lateral_range = 10.0;
const double lateral_spacing = 0.6;

bool ReferenceLineProcessor::solve(std::shared_ptr<ReferenceLine> reference_line_out, std::shared_ptr<FreeSpace> free_space_out) {
    if (not b_spline(reference_line_out)) {
        return false;
    }
    return search(reference_line_out, free_space_out);
}

bool ReferenceLineProcessor::b_spline(std::shared_ptr<ReferenceLine> reference_line_out) const {
    // B spline smoothing.
    double length = 0;
    for (size_t i = 0; i != _reference_line_points.size() - 1; ++i) {
        length += distance(_reference_line_points[i], _reference_line_points[i + 1]);
    }
    int degree = 3;
    double average_length = length / (_reference_line_points.size() - 1);
    if (average_length > 10) degree = 3;
    else if (average_length > 5) degree = 4;
    else degree = 5;

    tinyspline::BSpline b_spline_raw(_reference_line_points.size(), 2, degree);
    std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i != _reference_line_points.size(); ++i) {
        ctrlp_raw[2 * (i)] = _reference_line_points[i].x;
        ctrlp_raw[2 * (i) + 1] = _reference_line_points[i].y;
    }
    b_spline_raw.setControlPoints(ctrlp_raw);
    double delta_t = 1.0 / length;
    double tmp_t = 0;
    std::vector<double> x_list, y_list, s_list;
    while (tmp_t < 1) {
        auto result = b_spline_raw.eval(tmp_t).result();
        x_list.emplace_back(result[0]);
        y_list.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    auto result = b_spline_raw.eval(1).result();
    x_list.emplace_back(result[0]);
    y_list.emplace_back(result[1]);
    s_list.emplace_back(0);
    for (size_t i = 1; i != x_list.size(); ++i) {
        double dis = sqrt(pow(x_list[i] - x_list[i - 1], 2) + pow(y_list[i] - y_list[i - 1], 2));
        s_list.emplace_back(s_list.back() + dis);
    }

    tk::spline x_s, y_s;
    x_s.set_points(s_list, x_list);
    y_s.set_points(s_list, y_list);
    reference_line_out->initialize(x_s, y_s, s_list.back());
    return true;
}

bool ReferenceLineProcessor::smooth(std::shared_ptr<ReferenceLine> reference_line_out) const {

}

void ReferenceLineProcessor::calculate_cost(std::vector<std::vector<DpPoint>> &samples,
        int layer_index,
        int lateral_index) {
    if (layer_index == 0) return;
    if (!samples[layer_index][lateral_index].is_feasible) return;

    static const double weight_ref_offset = 1.0;
    static const double weight_obstacle = 0.5;
    static const double weight_angle_change = 16.0;
    static const double weight_ref_angle_diff = 0.5;
    static const double safe_distance = 3.0;

    auto &point = samples[layer_index][lateral_index];
    double self_cost = 0;
    if (point.dis_to_obs < safe_distance) self_cost += (safe_distance - point.dis_to_obs) / safe_distance * weight_obstacle;
    self_cost += fabs(point.l) / max_lateral_range * weight_ref_offset;

    auto min_cost = DBL_MAX;
    for (const auto &pre_point : samples[layer_index - 1]) {
        if (!pre_point.is_feasible) continue;
        if (fabs(pre_point.l - point.l) > (point.s - pre_point.s)) continue;
        double direction = atan2(point.y - pre_point.y, point.x - pre_point.x);
        double edge_cost = fabs(constrainAngle(direction - pre_point.dir)) / M_PI_2 * weight_angle_change
            + fabs(constrainAngle(direction - point.heading)) / M_PI_2 * weight_ref_angle_diff;
        double total_cost = self_cost + edge_cost + pre_point.cost;
        if (total_cost < min_cost) {
            min_cost = total_cost;
            point.parent = &pre_point;
            point.dir = direction;
        }
    }

    if (point.parent) point.cost = min_cost;
}

bool ReferenceLineProcessor::search(std::shared_ptr<ReferenceLine> reference_line_out, std::shared_ptr<FreeSpace> free_space_out) {
    if (not reference_line_out->is_initialized()) {
        return false;
    }

    // const tk::spline &x_s = reference_line_out->x_s();
    // const tk::spline &y_s = reference_line_out->y_s();
    // Sampling interval.
    const auto init_sl = reference_line_out->get_projection(XYPosition{_init_point.x, _init_point.y});
    double tmp_s = init_sl.s;
    layers_s_list_.clear();
    layers_bounds_.clear();
    double search_ds = reference_line_out->length() > 6.0 ? 1.5 : 0.5;
    while (tmp_s < reference_line_out->length()) {
        layers_s_list_.emplace_back(tmp_s);
        tmp_s += search_ds;
    }
    layers_s_list_.emplace_back(reference_line_out->length());
    if (layers_s_list_.empty()) return false;

    double vehicle_s = layers_s_list_.front();

    if (fabs(init_sl.l) > 5.0) {
        LOG(ERROR) << "Vehicle far from ref, quit graph search. Init l " << init_sl.l;
        return false;
    }
    int start_lateral_index =
        static_cast<int>((max_lateral_range + init_sl.l) / lateral_spacing);

    std::vector<std::vector<DpPoint>> samples;
    samples.reserve(layers_s_list_.size());
    const double search_threshold = 0.7;
    // Sample nodes.
    for (int i = 0; i < layers_s_list_.size(); ++i) {
        samples.emplace_back(std::vector<DpPoint>());
        double cur_s = layers_s_list_[i];
        // double ref_x = x_s(cur_s);
        // double ref_y = y_s(cur_s);
        // double ref_heading = getHeading(x_s, y_s, cur_s);
        // double ref_curvature = getCurvature(x_s, y_s, cur_s);
        const auto ref_pt = reference_line_out->get_reference_point(cur_s);
        double ref_r = 1 / ref_pt.kappa;
        double cur_l = -max_lateral_range;
        int lateral_index = 0;
        while (cur_l <= max_lateral_range) {
            DpPoint dp_point;
            dp_point.x = ref_pt.x + cur_l * cos(ref_pt.theta + M_PI_2);
            dp_point.y = ref_pt.y + cur_l * sin(ref_pt.theta + M_PI_2);
            dp_point.heading = ref_pt.theta;
            dp_point.s = cur_s;
            dp_point.l = cur_l;
            dp_point.layer_index = i;
            dp_point.lateral_index = lateral_index;
            grid_map::Position node_pose(dp_point.x, dp_point.y);
            dp_point.dis_to_obs = _map.isInside(node_pose) ? _map.getObstacleDistance(node_pose) : -1;
            if ((ref_pt.kappa < 0 && cur_l < ref_r) || (ref_pt.kappa > 0 && cur_l > ref_r)
                || dp_point.dis_to_obs < search_threshold) {
                dp_point.is_feasible = false;
            }
            if (i == 0 && dp_point.lateral_index != start_lateral_index) dp_point.is_feasible = false;
            if (i == 0 && dp_point.lateral_index == start_lateral_index) {
                dp_point.is_feasible = true;
                dp_point.dir = _init_point.theta;
                dp_point.cost = 0.0;
            }
            samples.back().emplace_back(dp_point);
            cur_l += lateral_spacing;
            ++lateral_index;
        }
        // Get rough bounds.
        auto &point_set = samples.back();
        for (int j = 0; j < point_set.size(); ++j) {
            if (j == 0 || !point_set[j - 1].is_feasible || !point_set[j].is_feasible) {
                point_set[j].rough_lower_bound = point_set[j].l;
            } else {
                point_set[j].rough_lower_bound = point_set[j - 1].rough_lower_bound;
            }
        }
        for (int j = point_set.size() - 1; j >= 0; --j) {
            if (j == point_set.size() - 1 || !point_set[j + 1].is_feasible || !point_set[j].is_feasible) {
                point_set[j].rough_upper_bound = point_set[j].l;
            } else {
                point_set[j].rough_upper_bound = point_set[j + 1].rough_upper_bound;
            }
        }
    }

    // Calculate cost.
    int max_layer_reached = 0;
    for (const auto &layer : samples) {
        bool is_layer_feasible = false;
        for (const auto &point : layer) {
            calculate_cost(samples, point.layer_index, point.lateral_index);
            if (point.parent) is_layer_feasible = true;
        }
        if (layer.front().layer_index != 0 && !is_layer_feasible) break;
        max_layer_reached = layer.front().layer_index;
    }

    // Retrieve path.
    const DpPoint *ptr = nullptr;
    auto min_cost = DBL_MAX;
    for (const auto &point : samples[max_layer_reached]) {
        if (point.cost < min_cost) {
            ptr = &point;
            min_cost = point.cost;
        }
    }

    while (ptr) {
        if (ptr->layer_index == 0) {
            layers_bounds_.emplace_back(-10, 10);
        } else {
            static const double check_s = 0.2;
            double upper_bound = check_s + ptr->rough_upper_bound;
            double lower_bound = -check_s + ptr->rough_lower_bound;
            static const double check_limit = 6.0;
            const auto ref_pt = reference_line_out->get_reference_point(ptr->s);
            while (upper_bound < check_limit) {
                grid_map::Position pos;
                pos(0) = ref_pt.x + upper_bound * cos(ptr->heading + M_PI_2);
                pos(1) = ref_pt.y + upper_bound * sin(ptr->heading + M_PI_2);
                if (_map.isInside(pos)
                    && _map.getObstacleDistance(pos) > check_s) {
                    upper_bound += check_s;
                } else {
                    upper_bound -= check_s;
                    break;
                }
            }
            while (lower_bound > -check_limit) {
                grid_map::Position pos;
                pos(0) = ref_pt.x + lower_bound * cos(ptr->heading + M_PI_2);
                pos(1) = ref_pt.y + lower_bound * sin(ptr->heading + M_PI_2);
                if (_map.isInside(pos)
                    && _map.getObstacleDistance(pos) > check_s) {
                    lower_bound -= check_s;
                } else {
                    lower_bound += check_s;
                    break;
                }
            }
            if (ref_pt.kappa > 0.0001) {
                upper_bound = std::min(upper_bound, 1.0 / ref_pt.kappa);
            } else if (ref_pt.kappa < -0.0001) {
                lower_bound = std::max(lower_bound, 1.0 / ref_pt.kappa);
            }
            layers_bounds_.emplace_back(lower_bound, upper_bound);
        }
        ptr = ptr->parent;
    }

    std::reverse(layers_bounds_.begin(), layers_bounds_.end());
    layers_s_list_.resize(layers_bounds_.size());

    free_space_out->set_reference_line(reference_line_out);
    free_space_out->mutable_boundary_points()->clear();
    for (std::size_t i = 0; i < layers_s_list_.size(); ++i) {
        const double s = layers_s_list_.at(i);
        const auto& bound = layers_bounds_.at(i);
        BoundaryPoint bd_pt;
        bd_pt.s = s;
        bd_pt.lb_l = bound.first;
        bd_pt.ub_l = bound.second;
        bd_pt.lb_xy = reference_line_out->get_xy_by_sl({s, bound.first});
        bd_pt.ub_xy = reference_line_out->get_xy_by_sl({s, bound.second});
        free_space_out->mutable_boundary_points()->emplace_back(bd_pt);
    }
    return true;
}

}