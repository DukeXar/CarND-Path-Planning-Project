#include "map.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "utils.h"

double Distance(double x1, double y1, double x2, double y2) {
  return sqrt(static_cast<long double>(x2 - x1) * (x2 - x1) +
              (y2 - y1) * (y2 - y1));
}

Point Map::FromFrenet(const FrenetPoint &pt, bool smooth) const {
  if (!m_splinesReady) {
    throw std::runtime_error("Freeze() must be called before");
  }

  const double s = ClampFrenetS(pt.s);
  const double x = m_splineX[smooth](s) + pt.d * m_splineDx[smooth](s);
  const double y = m_splineY[smooth](s) + pt.d * m_splineDy[smooth](s);
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

double Map::ClampFrenetS(double s) const {
  // Wrap if it is next lap.
  if (s > m_maxS) {
    return std::fmod(s, m_maxS);
  }

  return s;
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
  // We need to close the curve, and to make proper interpolation,
  // add a duplicate of the starting point, but with different S.
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
  m_maxS = m_waypointsFn.rbegin()->s;
}
