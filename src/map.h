#pragma once

#include <string>
#include <vector>
#include "spline.h"

struct Point {
  double x, y;
};

struct FrenetPoint {
  double s, d;
};

// Curve point - position S in Frenet, normal vector (dx, dy)
struct CurvePoint {
  double s, dx, dy;
};

double Distance(double x1, double y1, double x2, double y2);

// This is not quite good map, as it assumes that map is circle and it has
// hardcoded length inside, but it should be okay for the project.
class Map {
 public:
  static const bool kDefaultSmooth = true;

  void AddWaypoint(const Point &pt, const CurvePoint &cp);
  void Freeze();

  Point FromFrenet(const FrenetPoint &pt, bool smooth = kDefaultSmooth) const;

  std::vector<Point> FromFrenet(const std::vector<FrenetPoint> &pt,
                                bool smooth = kDefaultSmooth) const;

  double ClampFrenetS(double s) const;

  size_t get_size() const { return m_waypointsFn.size(); }

  const std::vector<Point> &get_waypoints_xy() const { return m_waypointsXY; }

  const std::vector<CurvePoint> &get_waypoints_fn() const {
    return m_waypointsFn;
  }

 private:
  std::vector<Point> m_waypointsXY;
  std::vector<CurvePoint> m_waypointsFn;
  tk::spline m_splineX[2], m_splineY[2], m_splineDx[2], m_splineDy[2];
  bool m_splinesReady = false;
  double m_maxS;
};

void ReadMap(const std::string &filename, Map &map);
