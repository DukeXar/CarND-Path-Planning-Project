#pragma once

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "map.h"
#include "trajectory.h"

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
  const std::unordered_map<int, std::vector<int>>& GetAllCarsByLane() const {
    return m_cars;
  }

 private:
  double m_laneWidth;
  std::unordered_map<int, std::vector<int>> m_cars;
  std::unordered_map<int, OtherCar> m_byId;
};

class World {
 public:
  World(const std::vector<OtherCarSensor>& sensors, double laneWidth);
  const Target& GetModelById(int id) const;
  WorldSnapshot Simulate(double time);

 private:
  double m_laneWidth;
  std::unordered_map<int, std::unique_ptr<Target>> m_models;
};

class Decider {
 public:
  Decider(double laneWidth, double minTrajectoryTimeSeconds,
          double latencySeconds, const Map& map);

  BestTrajectories ChooseBestTrajectory(
      const State2D& startState, const std::vector<OtherCarSensor>& sensors);

 private:
  double GetSafeLaneOffset() const;
  std::pair<double, double> GetSafeLaneOffsets(int laneIdx) const;

  double LimitAccelerationAndSpeed(const PolyFunction& sTraj,
                                   const PolyFunction& dTraj,
                                   double targetTime) const;

  BestTrajectories BuildLaneSwitchTrajectory(const State2D& startState,
                                             int targetLane, double targetSpeed,
                                             World& world);
  BestTrajectories BuildKeepDistanceTrajectory(const State2D& startState,
                                               int followingCarId,
                                               const WorldSnapshot& snapshot);
  BestTrajectories BuildKeepSpeedTrajectory(const State2D& startState,
                                            double targetSpeed);

 private:
  double m_horizonSeconds;
  double m_laneWidth;
  double m_minTrajectoryTimeSeconds;
  double m_latencySeconds;
  const Map& m_map;

  int m_state;

  int m_followingCarId;
  int m_currentLane;
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
                            const std::vector<OtherCarSensor>& sensors);

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
  std::chrono::high_resolution_clock::time_point m_prevUpdateTime;
};
