#pragma once

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "map.h"
#include "trajectory.h"

struct Car {
  Point pos;
  double yaw;
  double speed;
};

struct CarEx {
  Car car;
  FrenetPoint fp;
};

struct OtherCarSensor {
  int id;
  Point pos;
  Point speed;
  FrenetPoint fnPos;
};

struct OtherCar {
  int id;
  double speed;
  FrenetPoint fnPos;
};

class WorldSnapshot {
 public:
  WorldSnapshot(const std::vector<OtherCar>& cars, double laneWidth);

  bool GetClosestCar(int laneIdx, double s, OtherCar* result) const;
  OtherCar GetCarById(int id) const;
  double GetLaneWidth() const { return m_laneWidth; }

  const std::unordered_map<int, OtherCar>& GetAllCars() const { return m_byId; }
  const std::unordered_multimap<int, int>& GetAllCarsByLane() const {
    return m_cars;
  }

 private:
  double m_laneWidth;
  std::unordered_multimap<int, int> m_cars;
  std::unordered_map<int, OtherCar> m_byId;
};

class World {
 public:
  World(const std::vector<OtherCarSensor>& sensors, double laneWidth);
  const WorldSnapshot& Simulate(double time);

 private:
  double m_laneWidth;
  std::unordered_map<int, std::unique_ptr<Target>> m_models;
  std::unordered_map<double, std::unique_ptr<WorldSnapshot>> m_snapshots;
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
