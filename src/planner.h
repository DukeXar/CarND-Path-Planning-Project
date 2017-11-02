#pragma once

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "map.h"
#include "trajectory.h"
#include "world.h"

struct Car {
  Point pos;
  double yaw;
  double speed;
};

struct CarEx {
  Car car;
  FrenetPoint fp;
};

class Decider {
 public:
  enum class Mode { kKeepingLane, kChangingLane };
  typedef std::unordered_map<int, std::pair<bool, OtherCar>> LaneToOccupancy;

 public:
  Decider(double laneWidth, double minTrajectoryTimeSeconds,
          double latencySeconds, const Map& map);

  BestTrajectories ChooseBestTrajectory(
      const State2D& startState, const std::vector<OtherCarSensor>& sensors);

  Mode GetMode() const { return m_mode; }

 private:
  double GetSafeLaneOffset() const;

  std::pair<double, double> GetSafeLaneOffsets(int laneIdx) const;

  double LimitAccelerationAndSpeed(const PolyFunction& sTraj,
                                   const PolyFunction& dTraj,
                                   double targetTime) const;

  BestTrajectories BuildChangingLaneTrajectory(const State2D& startState,
                                               int sourceLane, int targetLane,
                                               double targetSpeed,
                                               World& world);
  BestTrajectories BuildKeepDistanceTrajectory(const State2D& startState,
                                               int followingCarId,
                                               double distanceToKeep,
                                               const WorldSnapshot& snapshot,
                                               World& world);
  BestTrajectories BuildKeepSpeedTrajectory(const State2D& startState,
                                            double targetSpeed, World& world);

  std::pair<bool, BestTrajectories> HandleKeepSpeedState(
      const State2D& startState, const WorldSnapshot& snapshot, World& world,
      const LaneToOccupancy& laneOccupancy);

  std::pair<bool, BestTrajectories> HandleChangingLaneState(
      const State2D& startState, const WorldSnapshot& snapshot, World& world,
      const LaneToOccupancy& laneOccupancy);

 private:
  double m_laneWidth;
  double m_minTrajectoryTimeSeconds;
  double m_latencySeconds;
  const Map& m_map;

  Mode m_mode;

  int m_currentLane;

  int m_followingCarId;
  int m_sourceLane;
  int m_targetLane;
  bool m_changingLeft;
  double m_targetSpeed;

  int m_updateNumber;
};

class Planner {
 public:
  explicit Planner(const Map& map);

  std::vector<Point> Update(const CarEx& car,
                            const std::vector<Point>& unprocessedPath,
                            const FrenetPoint& endPath,
                            const std::vector<OtherCarSensor>& sensors);

 private:
  const double m_updatePeriod;
  const double m_laneWidth;
  Map m_map;
  Decider m_decider;

  struct FullState {
    State2D fn;
    Point pt;
  };

  std::vector<FullState> m_plannedPath;

  unsigned m_updateNumber;
  std::chrono::high_resolution_clock::time_point m_prevUpdateTime;
};
