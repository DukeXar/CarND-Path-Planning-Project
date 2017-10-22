#include "planner.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include "Dense"
#include "map.h"
#include "utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

int DPosToCurrentLane(double d, double laneWidth) { return d / laneWidth; }

double CurrentLaneToDPos(int laneIdx, double laneWidth) {
  return laneIdx * laneWidth + laneWidth / 2;
}

class FixedTarget : public Target {
 public:
  explicit FixedTarget(const State2D& state) : m_state(state) {}
  virtual State2D At(double time) const override;

 private:
  State2D m_state;
};

State2D FixedTarget::At(double time) const { return m_state; }

class ConstantSpeedTarget : public Target {
 public:
  explicit ConstantSpeedTarget(double speed, double startS, const State& stateD,
                               double distance = 0, double latency = 0)
      : m_speed(speed),
        m_startS(startS),
        m_stateD(stateD),
        m_distance(distance),
        m_latency(latency) {}

  virtual State2D At(double time) const override;

 private:
  double m_speed;
  double m_startS;
  State m_stateD;
  double m_distance;
  double m_latency;
};

State2D ConstantSpeedTarget::At(double time) const {
  if (time < 0) {
    throw std::runtime_error("WHOA!");
  }
  double s = m_speed * (time + m_latency) + m_startS - m_distance;
  return State2D{State{s, m_speed, 0}, m_stateD};
}

WorldSnapshot::WorldSnapshot(const std::vector<OtherCar>& sensors,
                             double laneWidth)
    : m_laneWidth(laneWidth) {
  for (const auto& car : sensors) {
    int lane = DPosToCurrentLane(car.fnPos.d, laneWidth);
    if (lane >= 0) {
      m_cars[lane].push_back(car.id);
      m_byId[car.id] = car;
      //      std::cout << "lane=" << lane << ", d=" << car.fnPos.d << ", s=" <<
      //      car.fnPos.s << std::endl;
    }
  }

  for (auto& laneAndCars : m_cars) {
    std::sort(laneAndCars.second.begin(), laneAndCars.second.end(),
              [this](int id1, int id2) {
                return m_byId.find(id1)->second.fnPos.s <
                       m_byId.find(id2)->second.fnPos.s;
              });
  }

  //  std::cout << "Cars by lane: \n";
  //  for (const auto & laneAndCars : m_cars) {
  //    std::cout << laneAndCars.first << "=[";
  //    for (const auto & car : laneAndCars.second) {
  //      double v = std::sqrt(car.speed.x * car.speed.x + car.speed.y *
  //      car.speed.y);
  //      std::cout << car.id << "(s=" << car.fnPos.s << ", v=" << v << ")" <<
  //      ",";
  //    }
  //    std::cout << "]\n";
  //  }
  //  std::cout << std::endl;
}

bool WorldSnapshot::GetClosestCar(int laneIdx, double s,
                                  OtherCar* result) const {
  auto pos = m_cars.find(laneIdx);
  if (pos == m_cars.end()) {
    return false;
  }

  const auto& carsInLane = pos->second;

  const OtherCar example{0, 0, {s, 0}};
  auto lb = std::lower_bound(carsInLane.begin(), carsInLane.end(), example,
                             [this](int id1, const OtherCar& c2) {
                               const OtherCar& c1 = m_byId.find(id1)->second;
                               return c1.fnPos.s < c2.fnPos.s;
                             });

  if (lb == carsInLane.end()) {
    return false;
  }

  if (result) {
    *result = m_byId.find(*lb)->second;
  }

  return true;
}

OtherCar WorldSnapshot::GetCarById(int id) const {
  auto pos = m_byId.find(id);
  if (pos == m_byId.end()) {
    throw std::runtime_error("Not found");
  }
  return pos->second;
}

std::vector<OtherCar> SensorsToCars(
    const std::vector<OtherCarSensor>& sensors) {
  std::vector<OtherCar> result;
  for (const auto& sensor : sensors) {
    double speed = std::sqrt(sensor.speed.x * sensor.speed.x +
                             sensor.speed.y * sensor.speed.y);
    OtherCar car{sensor.id, speed, sensor.fnPos};
    result.push_back(car);
  }
  return result;
}

World::World(const std::vector<OtherCarSensor>& sensors, double laneWidth)
    : m_laneWidth(laneWidth) {
  WorldSnapshot current(SensorsToCars(sensors), m_laneWidth);
  for (const auto& idAndCar : current.GetAllCars()) {
    double latency = 0;
    const auto& car = idAndCar.second;
    const auto dstate = State{car.fnPos.d, 0, 0};
    m_models[car.id] = std::unique_ptr<Target>(
        new ConstantSpeedTarget(car.speed, car.fnPos.s, dstate, 0, latency));
  }
}

WorldSnapshot World::Simulate(double time) {
  std::vector<OtherCar> cars;
  for (auto& idAndModel : m_models) {
    auto state = idAndModel.second->At(time);
    OtherCar car{idAndModel.first, state.s.v,
                 FrenetPoint{state.s.s, state.d.s}};
    cars.push_back(car);
  }
  return WorldSnapshot(cars, m_laneWidth);
}

////////////////////////////////////////////////////////////////////////////////

typedef std::vector<std::pair<double, CostFunction>> WeightedFunctions;

double WeightedCostFunction(const WeightedFunctions& weigtedFunctions,
                            const PolyFunction& sTraj,
                            const PolyFunction& dTraj, double targetTime) {
  double result = 0;
  for (const auto& wf : weigtedFunctions) {
    result += wf.first * wf.second(sTraj, dTraj, targetTime);
  }
  return result;
}

double AllGoodFunction(const PolyFunction& sTraj, const PolyFunction& dTraj,
                       double targetTime) {
  return 1.0;
}

double Logistic(double x) { return 2.0 / (1 + exp(-x)) - 1.0; }

double ClosenessCost(double x1, double x2, double sigma) {
  return Logistic(std::abs(x1 - x2) / sigma);
}

double SpeedLimitCost(const PolyFunction& sTraj, const PolyFunction& dTraj,
                      double targetTime, double speedLimit) {
  double maxSpeedSoFar = 0;
  size_t totalIntervals = 100;
  double intervalLength = targetTime / totalIntervals;
  for (int i = 1; i < totalIntervals; ++i) {
    double speed = sTraj.Eval2(i * intervalLength);
    maxSpeedSoFar = std::max(maxSpeedSoFar, speed);
  }

  if (maxSpeedSoFar > speedLimit) {
    return 1.0;
  }

  double percent = 0.9;
  double threshold = speedLimit * percent;

  if (maxSpeedSoFar < threshold) {
    return 0.0;
  }

  // linear case
  //  double a = (1-percent) * speedLimit;
  //  double k = speedLimit / a;
  //  return k * maxSpeedSoFar - speedLimit;

  // power case
  double x = (maxSpeedSoFar - threshold) / (speedLimit - threshold);
  return std::pow(x, 7);
}

double AccelerationLimitCost(const PolyFunction& sTraj,
                             const PolyFunction& dTraj, double targetTime,
                             double accLimit) {
  // TODO this is a copy-paste from speed limit
  double maxAccSoFar = 0;
  size_t totalIntervals = 100;
  double intervalLength = targetTime / totalIntervals;
  for (int i = 1; i < totalIntervals; ++i) {
    double acc = sTraj.Eval3(i * intervalLength);
    maxAccSoFar = std::max(maxAccSoFar, acc);
  }

  if (maxAccSoFar > accLimit) {
    return 1.0;
  }

  double percent = 0.9;

  if (maxAccSoFar < accLimit * percent) {
    return 0.0;
  }

  double a = (1 - percent) * accLimit;
  double k = accLimit / a;
  return k * maxAccSoFar - accLimit;
}

std::pair<double, double> GetMaxCartesianAccelerationAndSpeed(
    const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime,
    const Map& map) {
  double maxAccSoFar = 0;
  double maxSpeedSoFar = 0;

  // TODO: This is so slow
  // size_t totalIntervals = 100;
  // double intervalLength = targetTime / totalIntervals;
  double intervalLength = 0.02;
  int totalIntervals = targetTime / intervalLength;

  std::vector<FrenetPoint> fnPoints;
  for (int i = 0; i < totalIntervals; ++i) {
    fnPoints.push_back(
        {sTraj.Eval(i * intervalLength), dTraj.Eval(i * intervalLength)});
  }
  std::vector<Point> points = map.FromFrenet(fnPoints);

  std::vector<double> speeds(points.size());
  std::vector<double> curves(points.size());

  double prevV = 0;
  double prevS = 0;

  double maxAccS = 0;

  double avgSpeedSum = 0;
  double avgSpeedCount = 0;
  double avgCurveSum = 0;
  double prevAvgSpeed = 0;
  bool hasPrevAvgSpeed = false;

  for (int i = 1; i < totalIntervals; ++i) {
    double distance =
        Distance(points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
    double v = distance / intervalLength;

    if (i > 1) {
      Point v1{points[i].x - points[i - 1].x, points[i].y - points[i - 1].y};
      Point v2{points[i - 1].x - points[i - 2].x,
               points[i - 1].y - points[i - 2].y};

      // This should be same as below.
      //      double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (prevS * distance);
      //      double turnRad = distance / sqrt(2 * (1 - cosPhi));
      //      double normalAcc = v * v / turnRad;

      double cosPhi = (v1.x * v2.x + v1.y * v2.y) /
                      (Distance(0, 0, v2.x, v2.y) * Distance(0, 0, v1.x, v1.y));
      Point v3{points[i].x - points[i - 2].x, points[i].y - points[i - 2].y};
      double curve = 2 * sqrt(1 - cosPhi * cosPhi) / Distance(0, 0, v3.x, v3.y);

      maxAccS = std::max(maxAccS, sTraj.Eval3((i - 1) * intervalLength));

      speeds[i] = v;
      curves[i] = curve;
      avgSpeedSum += v;
      avgCurveSum += curve;
      ++avgSpeedCount;

      // Average same way as simulator does, otherwise it gets inconsistent.
      if (avgSpeedCount > 10) {
        --avgSpeedCount;
        avgSpeedSum -= speeds[i - avgSpeedCount];
        avgCurveSum -= curves[i - avgSpeedCount];

        double avgSpeed = avgSpeedSum / avgSpeedCount;

        if (hasPrevAvgSpeed) {
          double avgCurve = avgCurveSum / avgSpeedCount;
          double normalAcc = avgSpeed * avgSpeed * avgCurve;
          double tangAcc =
              (avgSpeed - prevAvgSpeed) / (avgSpeedCount * intervalLength);
          double fullAcc = sqrt(normalAcc * normalAcc + tangAcc * tangAcc);
          maxAccSoFar = std::max(maxAccSoFar, fullAcc);
        }

        prevAvgSpeed = avgSpeed;
        hasPrevAvgSpeed = true;

        maxSpeedSoFar = std::max(maxSpeedSoFar, avgSpeed);

        avgSpeedCount = 1;
        avgSpeedSum = speeds[i];
        avgCurveSum = curves[i];
      }

      //      double normalAcc = v * v * curve;
      //      double tangAcc = (v - prevV) / intervalLength;
      //      double fullAcc = sqrt(normalAcc * normalAcc + tangAcc * tangAcc);
      //      maxAccSoFar = std::max(maxAccSoFar, fullAcc);
    }

    prevV = v;
    prevS = distance;
  }

  //  std::cout << "maxAccS=" << maxAccS << ", maxAcc=" << maxAccSoFar << "\n";
  return {maxAccSoFar, maxSpeedSoFar};
}

double PowerLimit(double x, double limit, double threshold) {
  if (x > limit) {
    return 100.0;
  }

  if (x < threshold) {
    return 0.0;
  }

  //  double a = accLimit - threshold;
  //  double k = accLimit / a;
  //  return k * maxAccSoFar - accLimit;

  double res = (x - threshold) / (limit - threshold);
  return std::pow(res, 7);
}

double CartesianAccelerationLimitCost(const PolyFunction& sTraj,
                                      const PolyFunction& dTraj,
                                      double targetTime, double accLimit,
                                      double speedLimit, const Map& map) {
  auto result =
      GetMaxCartesianAccelerationAndSpeed(sTraj, dTraj, targetTime, map);
  double maxAccSoFar = result.first;
  double maxSpeedSoFar = result.second;

  double accCost = PowerLimit(maxAccSoFar, accLimit, 0.9 * accLimit);
  double speedCost = PowerLimit(maxSpeedSoFar, speedLimit, 0.9 * speedLimit);

  return (10 * accCost + speedCost) / 11;
}

double CollidesWithAnyVehicle(const PolyFunction& sTraj,
                              const PolyFunction& dTraj, double targetTime,
                              double worldTimeOffset, World& world,
                              const Map& map, double distanceLimit,
                              const std::vector<int>& lanesToCheck,
                              bool verbose = false) {
  double intervalLength = 0.02;
  int totalIntervals = targetTime / intervalLength;

  double minDistance = std::numeric_limits<double>::max();

  for (int i = 0; i < totalIntervals; ++i) {
    double currentTime = i * intervalLength;
    FrenetPoint carPosFn = {sTraj.Eval(currentTime), dTraj.Eval(currentTime)};
    const auto carPos = map.FromFrenet(carPosFn);
    WorldSnapshot snapshot = world.Simulate(currentTime + worldTimeOffset);

    std::vector<int> laneIndices = lanesToCheck;
    if (lanesToCheck.empty()) {
      for (const auto& laneAndCars : snapshot.GetAllCarsByLane()) {
        laneIndices.push_back(laneAndCars.first);
      }
    }

    for (int laneIdx : laneIndices) {
      const auto& carIds = snapshot.GetAllCarsByLane().find(laneIdx)->second;
      for (const auto& carId : carIds) {
        const auto& car = snapshot.GetCarById(carId);
        const auto otherCarPos = map.FromFrenet(car.fnPos);
        double distance =
            Distance(carPos.x, carPos.y, otherCarPos.x, otherCarPos.y);

        if (verbose) {
          std::cout << "Checking " << car.id << ", s=" << car.fnPos.s
                    << ", d=" << car.fnPos.d << ", distance=" << distance
                    << std::endl;
        }

        minDistance = std::min(minDistance, distance);
      }
    }
  }

  if (minDistance <= distanceLimit) {
    return 1.0;
  }

  return 0.0;
}

double ClosenessToTargetSState(const PolyFunction& sTraj,
                               const PolyFunction& dTraj, const Target& target,
                               double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;

  cost += ClosenessCost(targetState.s.s, sTraj.Eval(targetTime), kSigmaSS);
  cost += ClosenessCost(targetState.s.v, sTraj.Eval2(targetTime), kSigmaSV);
  cost += ClosenessCost(targetState.s.acc, sTraj.Eval3(targetTime), kSigmaSAcc);

  return cost;
}

double ClosenessToTargetDState(const PolyFunction& sTraj,
                               const PolyFunction& dTraj, const Target& target,
                               double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;

  cost += ClosenessCost(targetState.d.s, dTraj.Eval(targetTime), kSigmaDS);
  cost += ClosenessCost(targetState.d.v, dTraj.Eval2(targetTime), kSigmaDV);
  cost += ClosenessCost(targetState.d.acc, dTraj.Eval3(targetTime), kSigmaDAcc);

  return cost;
}

double OutsideOfTheRoadPenalty(const PolyFunction& sTraj,
                               const PolyFunction& dTraj, double roadLeft,
                               double roadRight, double targetTime) {
  double step = 0.02;
  size_t totalIntervals = targetTime / step;

  bool failed = false;
  for (int i = 0; i < totalIntervals; ++i) {
    double d = dTraj.Eval(i * step);
    if (d <= roadLeft || d >= roadRight) {
      failed = true;
      break;
    }
  }

  return failed ? 1.0 : 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// class StateInterface {
// public:
//  virtual std::pair<PolyFunction, PolyFunction> ChooseTrajectory(const State2D
//  & startState,
//                                                                 const
//                                                                 std::vector<OtherCar>
//                                                                 & sensors);
//
//};

// README: Also the car should not experience total acceleration over 10 m/s^2
// and jerk that is greater than 50 m/s^3.
const double kMaxAccelerationMs2 = 10;
const double kMaxSpeedMs = MiphToMs(50);
const double kOtherVehicleMonitorDistance = 50;  // 100 was too much

const int kKeepSpeedState = 0;
const int kFollowVehicleState = 1;
const int kChangingLaneLeftState = 2;
const int kChangingLaneRightState = 3;

Decider::Decider(double laneWidth, double minTrajectoryTimeSeconds,
                 double latencySeconds, const Map& map)
    : m_laneWidth(laneWidth),
      m_minTrajectoryTimeSeconds(minTrajectoryTimeSeconds),
      m_latencySeconds(latencySeconds),
      m_map(map),
      m_state(kKeepSpeedState),
      m_followingCarId(-1),
      m_currentLane(-1),
      m_targetLane(-1),
      m_targetSpeed(0),
      m_updateNumber(0) {}

double Decider::GetSafeLaneOffset() const {
  return m_laneWidth / 2 - m_laneWidth / 8;
}

std::pair<double, double> Decider::GetSafeLaneOffsets(int laneIdx) const {
  double laneD = CurrentLaneToDPos(laneIdx, m_laneWidth);
  double safeLaneOffset = GetSafeLaneOffset();
  return {laneD - safeLaneOffset, laneD + safeLaneOffset};
}

BestTrajectories Decider::BuildLaneSwitchTrajectory(const State2D& startState,
                                                    int targetLane,
                                                    double targetSpeed,
                                                    World& world) {
  const double minDistanceM = 10;

  const double safeLaneOffset = GetSafeLaneOffset();
  const bool switchingToRight = targetLane > m_currentLane;
  const double targetLaneD = CurrentLaneToDPos(targetLane, m_laneWidth);
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  const auto sourceLaneOffsets = GetSafeLaneOffsets(m_currentLane);
  const auto targetLaneOffsets = GetSafeLaneOffsets(targetLane);

  const double leftBoundary =
      std::min(sourceLaneOffsets.first, targetLaneOffsets.first);
  const double rightBoundary =
      std::max(sourceLaneOffsets.second, targetLaneOffsets.second);

  std::cout << "Switching from lane " << m_currentLane << " to lane "
            << targetLane << ", boundary [" << leftBoundary << ", "
            << rightBoundary << std::endl;

  ConstantSpeedTarget target(targetSpeed, startState.s.s,
                             State{targetLaneD, 0, 0}, 0, 0);

  auto reactionTimePenalty = [](const PolyFunction& sTraj,
                                const PolyFunction& dTraj,
                                double targetTime) { return targetTime; };

  auto stayInLanesPenalty = [leftBoundary, rightBoundary](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {
    return OutsideOfTheRoadPenalty(sTraj, dTraj, leftBoundary, rightBoundary,
                                   targetTime);
  };

  auto collidesWithAnyVehicle = [this, &world, minDistanceM, targetLane](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {
    return CollidesWithAnyVehicle(sTraj, dTraj, targetTime, m_latencySeconds,
                                  world, m_map, minDistanceM,
                                  std::vector<int>{targetLane, m_currentLane});
  };

  WeightedFunctions weighted{
      {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
      {10, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      //{1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
      {500, stayInLanesPenalty},
      {500, collidesWithAnyVehicle},
      // TODO check distance to the vehicle in the lane
      {20, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
      {1, reactionTimePenalty}};

  auto costFunction = std::bind(WeightedCostFunction, weighted, _1, _2, _3);

  GenConfig cfg;
  cfg.sigmaS.s = kSigmaSS;
  cfg.sigmaS.v = kSigmaSV;
  cfg.sigmaS.acc = kSigmaSAcc;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 10;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  cfg.maxTime = 5;
  cfg.timeStep = 0.2;

  auto result = FindBestTrajectories(startState, target, cfg, costFunction);
  // std::cout << "--- Checking collide in lane " << targetLane << std::endl;
  // double cost = CollidesWithAnyVehicle(
  //     result.s, result.d, result.time, m_latencySeconds, world, m_map,
  //     minDistanceM, std::vector<int>{targetLane}, true);
  // std::cout << "--- Collide cost=" << cost << std::endl;
  return result;
}

BestTrajectories Decider::BuildKeepDistanceTrajectory(
    const State2D& startState, int followingCarId,
    const WorldSnapshot& snapshot) {
  OtherCar otherCar = snapshot.GetCarById(followingCarId);

  // TODO: this should be based on the target vehicle speed, maximum allowed
  // acceleration to go to full stop
  // s = 0.5/a * v^2
  // lets do 2x
  double distanceToKeep =
      2 * (otherCar.speed * otherCar.speed) * 0.5 / kMaxAccelerationMs2;

  std::cout << "Following vehicle id=" << followingCarId
            << ", distance=" << (otherCar.fnPos.s - startState.s.s)
            << ", distanceToKeep=" << distanceToKeep << std::endl;

  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  // TODO: should actually use world to do such simulation
  ConstantSpeedTarget target(
      otherCar.speed, otherCar.fnPos.s, State{currentLaneD, 0, 0},
      distanceToKeep,
      0);  // no need for latency, as it was incorporated by snapshot

  const auto safeOffsets = GetSafeLaneOffsets(m_currentLane);

  WeightedFunctions weighted{
      {30, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
      {1, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      //{90, std::bind(speedLimit, _1, _2, _3)},
      //{200, std::bind(accelerationLimit, _1, _2, _3)},
      //{1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
      {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, safeOffsets.first,
                      safeOffsets.second, _3)},
      //{5, reactionTimePenalty},
      {300, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
  };

  auto costFunction = std::bind(WeightedCostFunction, weighted, _1, _2, _3);

  GenConfig cfg;
  cfg.sigmaS.s = kSigmaSS;
  cfg.sigmaS.v = kSigmaSV;
  cfg.sigmaS.acc = kSigmaSAcc;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 40;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  cfg.maxTime = 25;
  cfg.timeStep = 0.2;

  return FindBestTrajectories(startState, target, cfg, costFunction);
}

BestTrajectories Decider::BuildKeepSpeedTrajectory(const State2D& startState,
                                                   double targetSpeed) {
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  ConstantSpeedTarget target(targetSpeed, startState.s.s,
                             State{currentLaneD, 0, 0}, 0, 0);

  const auto safeOffsets = GetSafeLaneOffsets(m_currentLane);

  WeightedFunctions weighted{
      {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
      {20, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      // {1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
      {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, safeOffsets.first,
                      safeOffsets.second, _3)},
      {50, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
  };

  auto costFunction = std::bind(WeightedCostFunction, weighted, _1, _2, _3);

  GenConfig cfg;
  cfg.sigmaS.s = 5;
  cfg.sigmaS.v = 20;
  cfg.sigmaS.acc = 4;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 40;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  cfg.maxTime = 7;
  cfg.timeStep = 0.2;

  return FindBestTrajectories(startState, target, cfg, costFunction);
}

double Decider::LimitAccelerationAndSpeed(const PolyFunction& sTraj,
                                          const PolyFunction& dTraj,
                                          double targetTime) const {
  return CartesianAccelerationLimitCost(
      sTraj, dTraj, targetTime, kMaxAccelerationMs2, kMaxSpeedMs, m_map);
}

BestTrajectories Decider::ChooseBestTrajectory(
    const State2D& startState, const std::vector<OtherCarSensor>& sensors) {
  ++m_updateNumber;
  m_currentLane = DPosToCurrentLane(startState.d.s, m_laneWidth);
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  if (m_targetLane == -1) {
    m_targetLane = m_currentLane;
  }

  World world(sensors, m_laneWidth);
  const auto snapshot = world.Simulate(m_latencySeconds);

  std::cout << "Cars by lane: \n";
  for (const auto& laneAndCarIds : snapshot.GetAllCarsByLane()) {
    std::cout << "Lane " << laneAndCarIds.first << "\n";
    for (const auto& carId : laneAndCarIds.second) {
      const auto& car = snapshot.GetCarById(carId);
      std::cout << "\t" << car.id << "\t" << car.speed << "\t" << car.fnPos.s
                << "\n";
    }
    std::cout << "\n";
  }
  std::cout << std::endl;

  if (m_state == kChangingLaneLeftState || m_state == kChangingLaneRightState) {
    bool areWeThereYet =
        (m_currentLane == m_targetLane) &&
        (std::abs(currentLaneD - startState.d.s) < m_laneWidth / 8);

    const char* dest = (m_state == kChangingLaneLeftState) ? "left" : "right";

    if (!areWeThereYet) {
      std::cout << "Changing lane " << dest << ", targetSpeed=" << m_targetSpeed
                << std::endl;
      return BuildLaneSwitchTrajectory(startState, m_targetLane, m_targetSpeed,
                                       world);
    }

    std::cout << "Reached the target " << dest << " lane" << std::endl;
  }

  typedef std::unordered_map<int, std::pair<bool, OtherCar>> LaneToOccupancy;
  LaneToOccupancy speeds;

  const int leftLaneIdx = std::max(0, m_currentLane - 1);
  const int rightLaneIdx = std::min(2, m_currentLane + 1);

  for (int laneIdx = leftLaneIdx; laneIdx <= rightLaneIdx; ++laneIdx) {
    OtherCar closestCar;
    if (snapshot.GetClosestCar(laneIdx, startState.s.s, &closestCar)) {
      speeds.insert({laneIdx, {true, closestCar}});
    } else {
      OtherCar stub;
      stub.speed = kMaxSpeedMs;
      stub.id = -1;
      speeds.insert({laneIdx, {false, stub}});
    }
  }

  std::cout << "Lanes status:\n";
  for (const auto& pp : speeds) {
    std::cout << pp.first << "\t occupied=" << pp.second.first
              << "\t speed=" << pp.second.second.speed << "\n";
  }
  std::cout << std::endl;

  auto maxSpeedLane = std::max_element(
      begin(speeds), end(speeds), [](const LaneToOccupancy::value_type& p1,
                                     const LaneToOccupancy::value_type& p2) {
        return p1.second.second.speed < p2.second.second.speed;
      });

  std::cout << "Max speed lane: " << maxSpeedLane->first << std::endl;

  if (maxSpeedLane->first == m_currentLane) {
    const auto& currentLane = speeds[m_currentLane];
    if (currentLane.first) {
      double distance = currentLane.second.fnPos.s - startState.s.s;
      if (distance > kOtherVehicleMonitorDistance) {
        m_state = kKeepSpeedState;
        m_followingCarId = -1;
      } else {
        m_state = kFollowVehicleState;
        m_followingCarId = currentLane.second.id;
      }
    } else {
      m_state = kKeepSpeedState;
      m_followingCarId = -1;
    }
  } else {
    const auto& currentLane = speeds[m_currentLane];
    if (currentLane.first) {
      double distance = currentLane.second.fnPos.s - startState.s.s;
      if (distance > kOtherVehicleMonitorDistance) {
        m_state = kKeepSpeedState;
        m_followingCarId = -1;
      } else {
        auto trajectory = BuildLaneSwitchTrajectory(
            startState, maxSpeedLane->first, startState.s.v, world);
        if (trajectory.cost < 200) {
          m_state = maxSpeedLane->first < m_currentLane
                        ? kChangingLaneLeftState
                        : kChangingLaneRightState;
          m_targetLane = maxSpeedLane->first;
          m_targetSpeed = startState.s.v;
          return trajectory;
        } else {
          std::cout << "Too hard to change lane - following car instead"
                    << std::endl;
          m_state = kFollowVehicleState;
          m_followingCarId = currentLane.second.id;
        }
      }
    } else {
      m_state = kKeepSpeedState;
      m_followingCarId = -1;
    }
  }

  if (m_state == kKeepSpeedState) {
    const double targetKeepSpeed = MiphToMs(48);
    return BuildKeepSpeedTrajectory(startState, targetKeepSpeed);
  }

  if (m_state == kFollowVehicleState) {
    return BuildKeepDistanceTrajectory(startState, m_followingCarId, snapshot);
  }

  throw std::runtime_error("Must not reach");

  /*
  switch (m_state) {
    case kKeepSpeedState: {
      OtherCar closestCar;
      if (snapshot.GetClosestCar(m_currentLane, startState.s.s, &closestCar)) {
        double distance = closestCar.fnPos.s - startState.s.s;

        std::cout << "Found closest car id=" << closestCar.id
                  << ", s=" << closestCar.fnPos.s << ", my s=" << startState.s.s
                  << std::endl;

        if (distance <= kOtherVehicleMonitorDistance) {
          m_state = kFollowVehicleState;
          m_followingCarId = closestCar.id;
        }
      }
      //      // This is to simulate lane change
      //      if (m_updateNumber == 9) {
      //        m_targetLane = m_targetLane - 1;
      //        m_state = kChangingLaneLeftState;
      //        m_targetSpeed = startState.s.v;
      //      }
    } break;

    case kFollowVehicleState: {
      OtherCar closestCar;
      if (snapshot.GetClosestCar(m_currentLane, startState.s.s, &closestCar)) {
        double distance = closestCar.fnPos.s - startState.s.s;

        if (distance > kOtherVehicleMonitorDistance) {
          std::cout << "No cars to follow"
                    << ", my s=" << startState.s.s << std::endl;
          m_state = kKeepSpeedState;
          m_followingCarId = -1;
        } else {
          if (m_followingCarId != closestCar.id) {
            std::cout << "Found new closest car id=" << closestCar.id
                      << ", s=" << closestCar.fnPos.s
                      << ", my s=" << startState.s.s << std::endl;
            m_followingCarId = closestCar.id;
          }
        }

        if (distance < 30) {
          if (m_currentLane > 1) {
            m_state = kChangingLaneLeftState;
            m_targetLane = m_currentLane - 1;
            m_targetSpeed = startState.s.v;
          } else if (m_currentLane < 2) {
            m_state = kChangingLaneRightState;
            m_targetLane = m_currentLane + 1;
            m_targetSpeed = startState.s.v;
          }
        }
      }
    } break;

    case kChangingLaneRightState:
    case kChangingLaneLeftState: {
    } break;

    default:
      throw std::runtime_error("Not implemented");
      break;
  };

  switch (m_state) {
    case kKeepSpeedState: {
      std::cout << "Keeping speed" << std::endl;
      const double targetKeepSpeed = MiphToMs(48);
      return BuildKeepSpeedTrajectory(startState, targetKeepSpeed);
    } break;

    case kFollowVehicleState: {
      auto trajectory =
          BuildKeepDistanceTrajectory(startState, m_followingCarId, snapshot);
      // FindBestTrajectories(startState, target, cfg, costFunction);
      // std::cout << "Future state: car pos=" << target.At(result.time).s.s
      //           << ", car speed=" << otherCar.speed
      //           << ", my pos=" << result.s.Eval(result.time)
      //           << ", my speed=" << result.s.Eval2(result.time) << std::endl;
      return trajectory;
    } break;

    case kChangingLaneLeftState: {
      std::cout << "Changing lane left, targetSpeed=" << m_targetSpeed
                << std::endl;
      auto trajectory = BuildLaneSwitchTrajectory(startState, m_targetLane,
                                                  m_targetSpeed, world);
      return trajectory;
    } break;

    case kChangingLaneRightState: {
      std::cout << "Changing lane right, targetSpeed=" << m_targetSpeed
                << std::endl;

      auto trajectory = BuildLaneSwitchTrajectory(startState, m_targetLane,
                                                  m_targetSpeed, world);
      return trajectory;
    } break;

    default:
      throw std::runtime_error("Not implemented");
      break;
  }

  throw std::runtime_error("Must not reach");
  */
}

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {
const double kReplanPeriodSeconds = 0.7;
const double kAlgorithmLatencySeconds = 0.4;
// 1 second latency of the algorithm, 50 points
const int kPointsToKeep = kAlgorithmLatencySeconds / 0.02;
const double kMaxUpdateLatencySeconds = 2;
const double kMinTrajectoryTimeSeconds =
    std::max(kReplanPeriodSeconds, kMaxUpdateLatencySeconds);
}  // namespace

Planner::Planner(const Map& map, double updatePeriodSeconds,
                 double laneWidthMeters)
    : m_updatePeriod(updatePeriodSeconds),
      m_laneWidth(laneWidthMeters),
      m_map(map),
      m_decider(laneWidthMeters, kMinTrajectoryTimeSeconds,
                kAlgorithmLatencySeconds, m_map),
      m_trajectoryOffsetIdx(0),
      m_hasTrajectory(false),
      m_updateNumber(0) {}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCarSensor>& sensors) {
  bool isTimeToReplan = m_updateNumber == 0;

  if (m_updateNumber == 0) {
    m_prevUpdateTime = std::chrono::high_resolution_clock::now();
  }

  ssize_t currentPosIdx = m_plannedPath.size() - unprocessedPath.size();

  bool continueTrajectory = true;

  // Handle huge lag.
  //  if (unprocessedPath.empty()) {
  //    continueTrajectory = false;
  //    currentPosIdx = 0;
  //    m_trajectoryOffsetIdx = 0;
  //    isTimeToReplan = true;
  //  }

  if (currentPosIdx * m_updatePeriod > kReplanPeriodSeconds) {
    isTimeToReplan = true;
  }

  //  std::cout << "idx=" << currentPosIdx << ", s=" << car.fp.s << ", d=" <<
  //  car.fp.d << ", speed=" << car.car.speed << ", phi=" << car.car.yaw <<
  //  (isTimeToReplan ? ", replanning" : ",") << std::endl;

  //  std::cout << "idx=" << currentPosIdx << ", s=" << car.fp.s << ", speed="
  //  << car.car.speed << ", up=" << unprocessedPath.size()
  //    << (isTimeToReplan ? ", replanning" : ",") << std::endl;

  if (!isTimeToReplan) {
    return unprocessedPath;
  }

  // Replan trajectories

  State2D startState{State{car.fp.s, car.car.speed, 0}, State{car.fp.d, 0, 0}};

  if (m_hasTrajectory && continueTrajectory) {
    ssize_t nextPosIdx = currentPosIdx + kPointsToKeep - m_trajectoryOffsetIdx;

    startState.s.s = m_plannedTrajectories.s.Eval(nextPosIdx * m_updatePeriod);
    startState.s.v = m_plannedTrajectories.s.Eval2(nextPosIdx * m_updatePeriod);
    startState.s.acc =
        m_plannedTrajectories.s.Eval3(nextPosIdx * m_updatePeriod);

    startState.d.s = m_plannedTrajectories.d.Eval(nextPosIdx * m_updatePeriod);
    startState.d.v = m_plannedTrajectories.d.Eval2(nextPosIdx * m_updatePeriod);
    startState.d.acc =
        m_plannedTrajectories.d.Eval3(nextPosIdx * m_updatePeriod);

    std::cout << "Start ";
    std::cout << "s=["
              << "s=" << startState.s.s << ", v=" << startState.s.v
              << ", acc=" << startState.s.acc << "], ";
    std::cout << "time=" << nextPosIdx * m_updatePeriod << "\n";
  }

  auto start = std::chrono::high_resolution_clock::now();
  auto bestTrajectory = m_decider.ChooseBestTrajectory(startState, sensors);
  using sec = std::chrono::duration<double>;
  auto delta = std::chrono::duration_cast<sec>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout
      << "Decider time=" << delta.count() << ", update time delta="
      << (std::chrono::duration_cast<sec>(start - m_prevUpdateTime).count() -
          currentPosIdx * m_updatePeriod)
      << "\n"
      << std::endl;
  m_prevUpdateTime = start;

  std::vector<Point> planned;
  if (continueTrajectory &&
      (currentPosIdx + kPointsToKeep - 1 < m_plannedPath.size())) {
    // append points from previous trajectory so that transition between
    // trajectories would be smooth
    planned.insert(planned.begin(), m_plannedPath.begin() + currentPosIdx,
                   m_plannedPath.begin() + currentPosIdx + kPointsToKeep);
    m_trajectoryOffsetIdx = planned.size();
  }

  std::vector<FrenetPoint> fnPoints;

  for (int i = 0; i < bestTrajectory.time / m_updatePeriod; ++i) {
    FrenetPoint pt = car.fp;
    pt.s = bestTrajectory.s.Eval(i * m_updatePeriod);
    pt.d = bestTrajectory.d.Eval(i * m_updatePeriod);
    fnPoints.push_back(pt);
  }

  // // Linear trajectory for testing purposes
  //  for (int i = 0; i < 5 / m_updatePeriod; ++i) {
  //    FrenetPoint pt = car.fp;
  //    pt.s += m_updatePeriod * i * 30;
  //    pt.d = CurrentLaneToDPos(1, m_laneWidth);
  //    //pt.s = bestTrajectory.s.Eval(i * m_updatePeriod);
  //    //pt.d = bestTrajectory.d.Eval(i * m_updatePeriod);
  //    fnPoints.push_back(pt);
  //    //planned.push_back(m_map.FromFrenetLinear(pt));
  //    planned.push_back(m_map.FromFrenet(pt));
  //  }

  std::vector<Point> newPlanned = m_map.FromFrenet(fnPoints);
  planned.insert(planned.end(), newPlanned.begin(), newPlanned.end());

  //  std::cout << "# " << m_updateNumber << std::endl;
  //  for (int i = 2; i < planned.size(); ++i) {
  //    double dist = Distance(planned[i-1].x, planned[i-1].y, planned[i].x,
  //    planned[i].y);
  //    //std::cout << plannedS[i] << "\t" << plannedS[i] - plannedS[i-1] <<
  //    "\t" << dist/m_updatePeriod << "\n";
  //    Point v1{planned[i].x - planned[i-1].x, planned[i].y - planned[i-1].y};
  //    Point v2{planned[i-1].x - planned[i-2].x, planned[i-1].y -
  //    planned[i-2].y};
  //
  //    double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (Distance(0, 0, v2.x,
  //    v2.y) * Distance(0, 0, v1.x, v1.y));
  //    double phi = std::acos(cosPhi);
  //    //double turnRad = prevS / sqrt(2 * (1 - cosPhi));
  //    //double normalAcc = prevV * prevV / turnRad;
  //    std::cout << "i=" << i << "\t" << rad2deg(phi) << ", x=" << planned[i].x
  //    << ", y=" << planned[i].y << std::endl;
  //  }
  //  std::cout << std::endl;

  m_plannedPath = planned;
  m_plannedTrajectories = bestTrajectory;
  m_hasTrajectory = true;
  m_updateNumber++;
  return m_plannedPath;
}
