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

class JerkMinimizingTrajectory {
public:
  JerkMinimizingTrajectory(const State & start, const State & end, double time)
  : m_start(start), m_end(end), m_time(time), m_fit(false) {
  }
  
  void Fit();
  
  double Eval(double time) const;
  double Eval2(double time) const;
  double Eval3(double time) const;
  
private:
  void AssertFit() const;
  
  State m_start;
  State m_end;
  double m_time;
  bool m_fit;
  // s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  std::array<double, 6> m_coeff;
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
  std::unique_ptr<JerkMinimizingTrajectory> m_plannedTrajectoryS, m_plannedTrajectoryD;
  unsigned m_updateNumber;
};
