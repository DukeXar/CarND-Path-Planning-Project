#include "map.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include "utils.h"
#include "spline_utils.h"

double Distance(double x1, double y1, double x2, double y2) {
  return sqrt(static_cast<long double>(x2 - x1) * (x2 - x1) +
              (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(const Point &point, const std::vector<Point> &map) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < map.size(); i++) {
    double dist = Distance(point.x, point.x, map[i].x, map[i].y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta,
                 const std::vector<Point> &map) {
  int closestWaypoint = ClosestWaypoint(Point{x, y}, map);

  auto pt = map[closestWaypoint];
  double heading = atan2((pt.y - y), (pt.x - x));
  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}

FrenetPoint ToFrenet(double x, double y, double theta,
                     const std::vector<Point> &map) {
  const int next_wp = NextWaypoint(x, y, theta, map);

  int prev_wp;
  if (next_wp == 0) {
    prev_wp = static_cast<int>(map.size() - 1);
  } else {
    prev_wp = next_wp - 1;
  }

  const auto prev_pt = map[prev_wp];
  const auto next_pt = map[next_wp];
  double n_x = next_pt.x - prev_pt.x;
  double n_y = next_pt.y - prev_pt.y;
  double x_x = x - prev_pt.x;
  double x_y = y - prev_pt.y;

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = Distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - prev_pt.x;
  double center_y = 2000 - prev_pt.y;
  double centerToPos = Distance(center_x, center_y, x_x, x_y);
  double centerToRef = Distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += Distance(map[i].x, map[i].y, map[i + 1].x, map[i + 1].y);
  }

  frenet_s += Distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

Point FromFrenet(const FrenetPoint &point,
                 const std::vector<CurvePoint> &map_fn,
                 const std::vector<Point> &map) {
  int prev_wp = -1;

  while (point.s > map_fn[prev_wp + 1].s &&
         (prev_wp < (int)(map_fn.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % map.size();
//  int wp3 = wp2 + 1;
//
//  using SplineUtils::GetSpline;
//
//  std::vector<double> s_points{map_fn[prev_wp].s, map_fn[wp2].s, map_fn[wp3].s};
//
//  std::cout << "points=" << s_points[0] << "," << s_points[1] << "," << s_points[2] << std::endl;
//
//  double d = -point.d;
//  auto spl_x = GetSpline(s_points, {map[prev_wp].x + map_fn[prev_wp].dx * d, map[wp2].x + map_fn[wp2].dx * d, map[wp3].x + map_fn[wp3].dx * d});
//  std::cout << "map.x=" << map[prev_wp].x << "," << map[wp2].x << "," << map[wp3].x << " -> " << spl_x(point.s) << std::endl;
//
//  auto spl_y = GetSpline(s_points, {map[prev_wp].y + map_fn[prev_wp].dy * d, map[wp2].y + map_fn[wp2].dy * d, map[wp3].y + map_fn[wp3].dy * d});
//  std::cout << "map.y=" << map[prev_wp].y << "," << map[wp2].y << "," << map[wp3].y << " -> " << spl_y(point.s) << std::endl;
//
//  double x = spl_x(point.s);
//  double x = spl_y(point.s);
  
  double heading =
      atan2((map[wp2].y - map[prev_wp].y), (map[wp2].x - map[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (point.s - map_fn[prev_wp].s);

  double seg_x = map[prev_wp].x + seg_s * cos(heading);
  double seg_y = map[prev_wp].y + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + point.d * cos(perp_heading);
  double y = seg_y + point.d * sin(perp_heading);

  return {x, y};
}

Point Map::FromFrenet(const FrenetPoint &pt) const {
  return ::FromFrenet(pt, m_waypointsFn, m_waypointsXY);
}

FrenetPoint Map::ToFrenet(const Car &car) const {
  return ::ToFrenet(car.pos.x, car.pos.y, car.yaw, m_waypointsXY);
}

int Map::ClosestWaypoint(const Point &pt) const {
  return ::ClosestWaypoint(pt, m_waypointsXY);
}

void Map::AddWaypoint(const Point &pt, const CurvePoint &cp) {
  m_waypointsXY.push_back(pt);
  m_waypointsFn.push_back(cp);
}

void ReadMap(const std::string &filename, Map &map) {
  std::ifstream in_map_(filename.c_str(), std::ifstream::in);
  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    Point pt;
    iss >> pt.x >> pt.y;
    CurvePoint cp;
    iss >> cp.s >> cp.dx >> cp.dy;
    map.AddWaypoint(pt, cp);
  }
}
