#pragma once

#include <algorithm>
#include <vector>
#include "spline.h"

namespace SplineUtils {
  struct Pt {
    double x, y;
    bool operator<(const Pt & p) const { return x < p.x; }
  };

  inline tk::spline GetSpline(const std::vector<double> & x, const std::vector<double> & y) {
    std::vector<Pt> pts;
    pts.reserve(x.size());
    for (int i = 0; i < x.size(); ++i) {
      pts.push_back(Pt{x[i], y[i]});
    }
    std::sort(pts.begin(), pts.end());
    std::vector<double> new_x, new_y;
    new_x.reserve(x.size());
    new_y.reserve(y.size());
    for (const auto & pt : pts) {
      new_x.push_back(pt.x);
      new_y.push_back(pt.y);
    }
    tk::spline spl;
    spl.set_points(new_x, new_y);
    return spl;
  }

} // namespace SplineUtils
