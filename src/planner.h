#pragma once

#include <chrono>
#include <memory>
#include <tuple>
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

  // This is a poor man's implementation of the 'memento' pattern.

  struct ModeParams {
    int currentLane;
    int followingCarId;
    int sourceLane;
    int targetLane;
    bool changingLeft;
    double targetSpeed;
    double lastLaneChangeS;
  };

  typedef std::unordered_map<int, std::pair<bool, OtherCar>> LaneToOccupancy;

 public:
  Decider(double laneWidth,
          double minTrajectoryTimeSeconds,
          double latencySeconds,
          const Map& map);

  std::vector<BestTrajectories> ChooseBestTrajectory(
      const State2D& startState,
      const std::vector<OtherCarSensor>& sensors,
      int currentTrajectoryIdx = -1);

 private:
  double GetSafeLaneOffset() const;

  std::pair<double, double> GetSafeLaneOffsets(int laneIdx) const;

  double LimitAccelerationAndSpeed(const PolyFunction& sTraj,
                                   const PolyFunction& dTraj,
                                   double targetTime) const;

  BestTrajectories BuildChangingLaneTrajectory(const State2D& startState,
                                               int sourceLane,
                                               int targetLane,
                                               double targetSpeed,
                                               World& world);
  BestTrajectories BuildKeepDistanceTrajectory(const State2D& startState,
                                               int followingCarId,
                                               double distanceToKeep,
                                               const WorldSnapshot& snapshot,
                                               World& world);
  BestTrajectories BuildKeepSpeedTrajectory(const State2D& startState,
                                            double targetSpeed,
                                            World& world);

  std::tuple<bool, ModeParams, BestTrajectories> HandleChangingLaneState(
      const State2D& startState,
      const WorldSnapshot& snapshot,
      World& world,
      const LaneToOccupancy& laneOccupancy,
      const ModeParams& params);

  std::tuple<Decider::Mode, ModeParams, BestTrajectories> ChooseBestTrajectory(
      const State2D& startState,
      World& world,
      double time,
      Mode mode,
      const ModeParams& params);

 private:
  double m_laneWidth;
  double m_minTrajectoryTimeSeconds;
  double m_latencySeconds;
  const Map& m_map;

  std::vector<std::tuple<Mode, ModeParams, BestTrajectories>> m_trajectories;

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
    int trajIdx;
  };

  std::vector<FullState> m_plannedPath;

  unsigned m_updateNumber;
  std::chrono::high_resolution_clock::time_point m_prevUpdateTime;
};
