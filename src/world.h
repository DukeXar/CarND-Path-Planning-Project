#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "map.h"
#include "trajectory.h"

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
  static const size_t kMaxLaneIdx = 3;

 public:
  WorldSnapshot(const std::vector<OtherCar>& cars, double laneWidth);

  bool GetClosestCar(int laneIdx, double s, OtherCar* result) const;
  OtherCar GetCarById(int id) const;
  double GetLaneWidth() const { return m_laneWidth; }

  const std::vector<int>& GetAllCarsByLane(int laneIdx) const {
    return m_cars[laneIdx];
  }

  size_t GetMaxLaneIdx() const { return kMaxLaneIdx; }

 private:
  double m_laneWidth;
  std::array<std::vector<int>, kMaxLaneIdx + 1> m_cars;
  std::vector<OtherCar> m_byId;
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

class ConstantSpeedTarget : public Target {
 public:
  explicit ConstantSpeedTarget(double speed,
                               double startS,
                               const State& stateD,
                               double distance = 0,
                               double latency = 0)
      : m_speed(speed),
        m_startS(startS),
        m_stateD(stateD),
        m_distance(distance),
        m_latency(latency) {}

  virtual State2D At(double time) const override;

 private:
  const double m_speed;
  const double m_startS;
  const State m_stateD;
  const double m_distance;
  const double m_latency;
};

class ConstantAccelerationTargetSpeedTarget : public Target {
 public:
  explicit ConstantAccelerationTargetSpeedTarget(double targetSpeed,
                                                 double acc,
                                                 double startS,
                                                 double startV,
                                                 const State& startD,
                                                 double distance = 0,
                                                 double latency = 0)
      : m_targetSpeed(targetSpeed),
        m_acc(acc),
        m_startS(startS),
        m_startV(startV),
        m_startD(startD),
        m_distance(distance),
        m_latency(latency) {}

  virtual State2D At(double time) const override;

 private:
  const double m_targetSpeed;
  const double m_acc;
  const double m_startS;
  const double m_startV;
  const State m_startD;
  const double m_distance;
  const double m_latency;
};
