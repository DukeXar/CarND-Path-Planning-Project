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

// This is not quite good map, as it assumes that map is circle and it has
// hardcoded length inside, but it should be okay for the project.
class Map {
 public:
  void AddWaypoint(const Point &pt, const CurvePoint &cp);
  void Freeze();

  Point FromFrenet(const FrenetPoint &pt, bool smooth = true) const;
  std::vector<Point> FromFrenet(const std::vector<FrenetPoint> &pt,
                                bool smooth = true) const;
  FrenetPoint ToFrenet(const Car &car) const;
  int ClosestWaypoint(const Point &pt) const;

  size_t GetSize() const { return m_waypointsFn.size(); }

 private:
  std::vector<Point> m_waypointsXY;
  std::vector<CurvePoint> m_waypointsFn;

  tk::spline m_splineX[2], m_splineY[2], m_splineDx[2], m_splineDy[2];
  bool m_splinesReady = false;
};

void ReadMap(const std::string &filename, Map &map);
