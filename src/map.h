#pragma once

#include <string>
#include <vector>
#include "spline.h"

struct Point {
  double x, y;
};

struct CurvePoint {
  double s, dx, dy;
};

struct FrenetPoint {
  double s, d;
};

struct Car {
  Point pos;
  double yaw;
  double speed;
};

double Distance(double x1, double y1, double x2, double y2);

class Map {
 public:
  void AddWaypoint(const Point &pt, const CurvePoint &cp);
  void Freeze();

  const std::vector<Point> &get_waypoints_xy() { return m_waypointsXY; }
  const std::vector<CurvePoint> &get_waypoints_fn() { return m_waypointsFn; }

  Point FromFrenet(const FrenetPoint &pt) const;
  std::vector<Point> FromFrenet(const std::vector<FrenetPoint> &pt) const;
  FrenetPoint ToFrenet(const Car &car) const;
  int ClosestWaypoint(const Point &pt) const;

  size_t GetSize() const { return m_waypointsFn.size(); }

 private:
  std::vector<Point> m_waypointsXY;
  std::vector<CurvePoint> m_waypointsFn;

  tk::spline m_splineX, m_splineY, m_splineDx, m_splineDy;
  bool m_splinesReady = false;
};

void ReadMap(const std::string &filename, Map &map);
