#pragma once

#include <memory>
#include <vector>
#include "map.h"
#include "trajectory.h"


class Decider {
public:
  Decider(double horizonSeconds, double laneWidth, double minTrajectoryTimeSeconds);

  std::pair<PolyFunction, PolyFunction> ChooseBestTrajectory(const State2D & startState, const std::vector<OtherCar> & sensors);
  
private:
  double m_horizonSeconds;
  double m_laneWidth;
  double m_minTrajectoryTimeSeconds;
  int m_state;
  
  int m_followingCarId;
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
  Decider m_decider;
  std::vector<Point> m_plannedPath;
  PolyFunction m_plannedTrajectoryS, m_plannedTrajectoryD;
  size_t m_trajectoryOffsetIdx;
  bool m_hasTrajectory;
  unsigned m_updateNumber;
};
