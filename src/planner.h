#pragma once

#include <memory>
#include <vector>
#include "map.h"
#include "trajectory.h"


class Decider {
public:
  Decider(double horizonSeconds, double laneWidth, double minTrajectoryTimeSeconds, const Map & map);

  BestTrajectories ChooseBestTrajectory(const State2D & startState, const std::vector<OtherCar> & sensors);
  
private:
  double m_horizonSeconds;
  double m_laneWidth;
  double m_minTrajectoryTimeSeconds;
  const Map & m_map;

  int m_state;
  
  int m_followingCarId;
  int m_targetLane;
  double m_targetSpeed;
  int m_updateNumber;
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
  BestTrajectories m_plannedTrajectories;
  size_t m_trajectoryOffsetIdx;
  bool m_hasTrajectory;
  unsigned m_updateNumber;
};
