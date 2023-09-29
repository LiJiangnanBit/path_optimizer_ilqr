#pragma once
#include <unordered_set>
#include <memory>
#include <queue>
#include "path/reference_line.h"
#include "path/free_space.h"
#include "path/data_structure.h"
#include "Map.hpp"

namespace Test {
using PathPlanning::PathPoint;
using PathPlanning::ReferenceLine;
using PathPlanning::FreeSpace;

// Point for A* search.
struct APoint {
    double x{};
    double y{};
    double s{};
    double l{};
    double g{};
    double h{};
    double dir{};
    // Layer denotes the index of the longitudinal layer that the point lies on.
    int layer{-1};
    int offset_idx{};
    double rough_upper_bound, rough_lower_bound;
    bool is_in_open_set{false};
    APoint *parent{nullptr};
    inline double f() {
        return g + h;
    }
};

// Point for DP.
struct DpPoint {
    double x, y, heading, s, l, cost = DBL_MAX, dir, dis_to_obs;
    int layer_index, lateral_index;
    double rough_upper_bound, rough_lower_bound;
    const DpPoint *parent = nullptr;
    bool is_feasible = true;
};

class PointComparator {
 public:
    bool operator()(APoint *a, APoint *b) {
        return a->f() > b->f();
    }
};

class ReferenceLineProcessor {
public:
    ReferenceLineProcessor(
        const std::vector<PathPlanning::PathPoint>& reference_line_points,
        const Map& map,
        const PathPoint& init_point) :
        _reference_line_points(reference_line_points),
        _map(map),
        _init_point(init_point) {};
    bool solve(std::shared_ptr<ReferenceLine> reference_line_out, std::shared_ptr<FreeSpace> free_space_out);
    bool b_spline(std::shared_ptr<ReferenceLine> reference_line_out) const;
    bool smooth(std::shared_ptr<ReferenceLine> reference_line_out) const;
    bool search(std::shared_ptr<ReferenceLine> reference_line_out, std::shared_ptr<FreeSpace> free_space_out);
private:
    void calculate_cost(std::vector<std::vector<DpPoint>> &samples,
        int layer_index,
        int lateral_index);

    std::vector<PathPlanning::PathPoint> _reference_line_points;
    const Map& _map;
    const PathPoint& _init_point;

    std::priority_queue<APoint *, std::vector<APoint *>, PointComparator> open_set_;
    std::unordered_set<const APoint *> closed_set_;
    std::vector<double> layers_s_list_;
    std::vector<std::pair<double, double>> layers_bounds_;
};

}