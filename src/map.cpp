#include "map.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "spline_utils.h"
#include "utils.h"

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

Point Map::FromFrenet(const FrenetPoint &pt, bool smooth) const {
  if (!m_splinesReady) {
    throw std::runtime_error("Freeze() must be called before");
  }
  double x = m_splineX[smooth](pt.s) + pt.d * m_splineDx[smooth](pt.s);
  double y = m_splineY[smooth](pt.s) + pt.d * m_splineDy[smooth](pt.s);
  return {x, y};
}

std::vector<Point> Map::FromFrenet(const std::vector<FrenetPoint> &points,
                                   bool smooth) const {
  if (!m_splinesReady) {
    throw std::runtime_error("Freeze() must be called before");
  }

  std::vector<Point> result;
  result.reserve(points.size());

  for (const auto &pt : points) {
    result.push_back(FromFrenet(pt, smooth));
  }

  return result;
}

FrenetPoint Map::ToFrenet(const Car &car) const {
  return ::ToFrenet(car.pos.x, car.pos.y, car.yaw, m_waypointsXY);
}

int Map::ClosestWaypoint(const Point &pt) const {
  return ::ClosestWaypoint(pt, m_waypointsXY);
}

void Map::AddWaypoint(const Point &pt, const CurvePoint &cp) {
  m_splinesReady = false;
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

  // TODO Beware! Ugly hack!
  CurvePoint cp = map.get_waypoints_fn()[0];
  cp.s = 6945.554;
  map.AddWaypoint(map.get_waypoints_xy()[0], cp);

  map.Freeze();
}

void Map::Freeze() {
  const size_t sz = m_waypointsXY.size();

  std::vector<double> flattenedX(sz, 0);
  std::vector<double> flattenedY(sz, 0);
  std::vector<double> flattenedS(sz, 0);
  std::vector<double> flattenedDx(sz, 0);
  std::vector<double> flattenedDy(sz, 0);

  for (int i = 0; i < sz; ++i) {
    flattenedX[i] = m_waypointsXY[i].x;
    flattenedY[i] = m_waypointsXY[i].y;
    flattenedS[i] = m_waypointsFn[i].s;
    flattenedDx[i] = m_waypointsFn[i].dx;
    flattenedDy[i] = m_waypointsFn[i].dy;
  }

  for (int i = 0; i < 2; ++i) {
    bool useCubic = i > 0;
    m_splineX[i].set_points(flattenedS, flattenedX, useCubic);
    m_splineY[i].set_points(flattenedS, flattenedY, useCubic);
    m_splineDx[i].set_points(flattenedS, flattenedDx, useCubic);
    m_splineDy[i].set_points(flattenedS, flattenedDy, useCubic);
  }

  m_splinesReady = true;
}
