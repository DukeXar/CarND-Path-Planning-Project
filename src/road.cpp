#include <iostream>
#include <algorithm>
#include "map.h"
#include "spline.h"

void PrintSpline(std::ostream & os, tk::spline spline, double x1, double x2, double step, int idx) {
  const char * kSep = ";";
  while (x1 <= x2) {
    os << idx << kSep << x1 << kSep << spline(x1) << "\n";
    x1 += step;
  }
}

int main() {
  const std::string MAP_FILENAME = "../data/highway_map.csv";
  Map map;
  ReadMap(MAP_FILENAME, map);
  auto waypoints = map.get_waypoints_xy();
  
  
  tk::spline spline;
  for (int i = 2; i < waypoints.size(); ++i) {
    std::vector<double> pts_x{waypoints[i-2].x, waypoints[i-1].x, waypoints[i].x};
    std::vector<double> pts_y{waypoints[i-2].y, waypoints[i-1].y, waypoints[i].y};
    spline.set_points(pts_x, pts_y);
    
    PrintSpline(std::cout, spline, waypoints[i-2].x, waypoints[i].x, 0.1, i-2);
  }

  return 0;
}
