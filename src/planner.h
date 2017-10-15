#pragma once

#include <memory>
#include <array>
#include <vector>
#include "map.h"

struct CarEx {
  Car car;
  FrenetPoint fp;
};

struct OtherCar {
  int id;
  Point pos;
  Point speed;
  FrenetPoint fnPos;
};

struct State {
  double s;
  double v;
  double acc;
};

class PolyFunction {
public:
  PolyFunction() {}
  explicit PolyFunction(const std::array<double, 6> & coeff): m_coeff(coeff) {}
  
  double Eval(double x) const;
  double Eval2(double x) const;
  double Eval3(double x) const;

private:
  std::array<double, 6> m_coeff;
};

class JerkMinimizingTrajectory {
public:
  JerkMinimizingTrajectory(const State & start, const State & end, double time)
  : m_start(start), m_end(end), m_time(time) {
  }

  PolyFunction Fit() const;

private:
  State m_start;
  State m_end;
  double m_time;
};

class Planner {
 public:
  explicit Planner(const Map& map, double updatePeriodS,
                   double laneWidthMeters);

  std::vector<Point> Update(const CarEx& car,
                            const std::vector<Point>& unprocessedPath,
                            const FrenetPoint& endPath,
                            const std::vector<OtherCar>& sensors);

 private:
  const double m_updatePeriod;
  const double m_laneWidth;
  Map m_map;
  std::vector<Point> m_plannedPath;
  PolyFunction m_plannedTrajectoryS, m_plannedTrajectoryD;
  size_t m_trajectoryOffsetIdx;
  bool m_hasTrajectory;
  unsigned m_updateNumber;
};
