#include "map.h"

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Dense"
#include "utils.h"

Point Map::FromFrenet(const FrenetPoint &pt, bool smooth) const {
  if (!m_splinesReady) {
    throw std::runtime_error("Freeze() must be called before");
  }

  const double s = ClampFrenetS(pt.s);
  // const double nx = m_splineY[smooth].deriv(1, s);
  // const double ny = -m_splineX[smooth].deriv(1, s);
  const double nx = m_splineDx[smooth](s);
  const double ny = m_splineDy[smooth](s);
  const double x = m_splineX[smooth](s) + pt.d * nx;
  const double y = m_splineY[smooth](s) + pt.d * ny;
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

FrenetPoint Map::ToFrenet(const Point &pt, double sStart) const {
  const double gamma = 0.001;
  const double precision = 1e-5;

  double s = ClampFrenetS(sStart);
  double prevStep = s;

  // clang-format off
  // f(s) = (pt.x - spline_x(s))^2 + (pt.y - spline_y(s)) ^ 2) ^ 0.5
  // u = (pt.x - spline_x(s)) ^ 2 + (pt.y - spline_y(s)) ^ 2
  // df(s)/ds = u ^ 0.5 = du(s)/ds / (2 * u ^ 0.5)
  // du(s)/ds = 2 * (pt.x - spline_x(s)) * (-spline_x.deriv(1, s)) + 2 * (pt.y - spline_y(s)) * (-spline_y.deriv(1, s))
  // 
  //            2 * (pt.x - spline_x(s)) * (-spline_x.deriv(1, s)) + 2 * (pt.y - spline_y(s)) * (-spline_y.deriv(1, s))
  // df(s)/ds = ------------------------------------------------------------------------------------------------------- =
  //            2 * ( (pt.x - spline_x(s)) ^ 2 + (pt.y - spline_y(s)) ^ 2 )^0.5
  //
  //       (pt.x - spline_x(s)) * spline_x.deriv(1, s) + (pt.y - spline_y(s)) * spline_y.deriv(1, s)
  // = -  ------------------------------------------------------------------------------------------------------ =
  //       ( (pt.x - spline_x(s)) ^ 2 + (pt.y - spline_y(s)) ^ 2 )^0.5
  //
  // : errX = pt.x - spline_x(s),
  // : errY = pt.y - spline_y(s)
  //
  //       errX * spline_x.deriv(1, s) + errY * spline_y.deriv(1, s)
  // = -  ----------------------------------------------------------- =
  //       (errX ^ 2 + errY ^ 2)^0.5
  // clang-format on

  while (prevStep > precision) {
    const auto prevS = s;
    const double errX = pt.x - m_splineX[1](s);
    const double errY = pt.y - m_splineY[1](s);
    const double df =
        -(errX * m_splineX[1].deriv(1, s) + errY * m_splineY[1].deriv(1, s)) /
        sqrt(errX * errX + errY * errY);
    s += -gamma * df;
    prevStep = std::abs(s - prevS);
  }

  const double nx = m_splineDx[1](s);
  const double ny = m_splineDy[1](s);
  const double dx = pt.x - m_splineX[1](s);
  const double dy = pt.y - m_splineY[1](s);
  double d = 0;

  if (nx == 0) {
    d = dy / ny;
  } else if (ny == 0) {
    d = dx / nx;
  } else {
    d = (dx / nx + dy / ny) / 2.0;
  }

  return FrenetPoint{s, d};
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
