#include "planner.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include "Dense"
#include "map.h"
#include "utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace {

const double kRefreshPeriodSeconds = 0.02;

const double kReplanPeriodSeconds = 0.5;
const double kAlgorithmLatencySeconds = 1.0;
// E.g. 1 second latency of the algorithm, 50 points
const int kPointsToKeep = kAlgorithmLatencySeconds / kRefreshPeriodSeconds;
// This is maximum time we think the planner can stuck - we should have
// trajectory for that time to keep moving.
const double kMaxUpdateLatencySeconds = 1;
// const double kMinTrajectoryTimeSeconds =
//     std::max(kReplanPeriodSeconds, kMaxUpdateLatencySeconds);

const double kMinTrajectoryTimeSeconds = 3;

// README: Also the car should not experience total acceleration over 10 m/s^2
// and jerk that is greater than 50 m/s^3.
const double kMaxAccelerationMs2 = 10;
const double kMaxSpeedMs = MiphToMs(49);
const double kTargetKeepSpeed = MiphToMs(48);
// 50 works pretty well with combination of number of samples
const double kOtherVehicleMonitorDistance = 50;
const double kMinToLaneBorderMeters = 0.8;
const double kLaneWidthMeters = 4;
const double kMaxLaneChangeTimeSeconds = 3;
const double kMinCarDistanceMeters = 10;
// The trajectory time depends on speed and distance. We monitor 50 meters in
// front (kOtherVehicleMonitorDistance), so that lets say with minimal speed of
// 10 m/s, it should be 5 seconds. It should be possible to automate this.
const double kMaxTrajectoryTimeToKeepDistanceSeconds = 7;

// Print CHECKPOINT failure when trajectory cost is higher than this value.
const int kHighCostCheckpoint = 300;

void DisplayCarsByLane(const WorldSnapshot& snapshot) {
  std::set<int> indices;
  for (const auto& laneAndCarIds : snapshot.GetAllCarsByLane()) {
    indices.insert(laneAndCarIds.first);
  }

  std::cout << "Cars by lane: \n";

  for (int laneIdx : indices) {
    const auto& laneAndCarIdsRange =
        snapshot.GetAllCarsByLane().equal_range(laneIdx);
    int col = 1;
    std::cout << "Lane " << laneAndCarIdsRange.first->first;
    std::cout << "\t id  speed  pos   |\n";
    for (auto it = laneAndCarIdsRange.first; it != laneAndCarIdsRange.second;
         ++it) {
      const auto carId = it->second;
      const auto& car = snapshot.GetCarById(carId);
      std::cout << "\t" << std::setw(3) << car.id << ' ';
      std::cout << std::setw(6) << std::setprecision(2) << std::fixed
                << car.speed << ' ';
      std::cout << std::setw(7) << std::setprecision(2) << std::fixed
                << car.fnPos.s << '|';
      if (col % 4 == 0) {
        std::cout << "\n";
      }
      ++col;
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

void DisplayLaneOccupancy(const Decider::LaneToOccupancy& speeds) {
  std::vector<int> indices;
  for (const auto& pp : speeds) {
    indices.push_back(pp.first);
  }
  std::sort(begin(indices), end(indices));

  std::cout << "Lanes status:\n";
  for (int idx : indices) {
    std::cout << "\t" << std::setw(6) << idx << "|";
  }
  std::cout << "\n";
  for (int idx : indices) {
    const auto& pp = *speeds.find(idx);
    std::cout << "\t";
    if (pp.second.first) {
      std::cout << "   XXX|";
    } else {
      std::cout << "   ...|";
    }
  }
  std::cout << "\n";
  for (int idx : indices) {
    const auto& pp = *speeds.find(idx);
    std::cout << "\t";
    std::cout << std::setw(6) << std::setprecision(2) << std::fixed
              << pp.second.second.speed << "|";
  }
  std::cout << std::endl;
}

double GetMinDistanceToKeep(double speed) {
  // s = 0.5/a * v^2
  double distanceToFullStop = speed * speed * 0.5 / kMaxAccelerationMs2;
  return distanceToFullStop;
}

double GetMaxTimeToStop(double speed) {
  // v = v0 - a * t = 0
  // t = v0 / a
  return speed / kMaxAccelerationMs2;
}

}  // namespace

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

double Logistic(double x) { return 2.0 / (1 + exp(-x)) - 1.0; }

double ClosenessCost(double x1, double x2, double sigma) {
  return Logistic(std::abs(x1 - x2) / sigma);
}

double GetCosAngle(const Point& p1, const Point& p2) {
  return (p1.x * p2.x + p1.y * p2.y) /
         (Distance(0, 0, p2.x, p2.y) * Distance(0, 0, p1.x, p1.y));
}

double GetAngle(const Point& p1, const Point& p2) {
  return std::acos(GetCosAngle(p1, p2));
}

std::pair<double, double> GetMaxCartesianAccelerationAndSpeed(
    const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime,
    const Map& map) {
  double maxAccSoFar = 0;
  double maxSpeedSoFar = 0;
  const double intervalLength = kRefreshPeriodSeconds;
  const int totalIntervals = targetTime / intervalLength;

  std::vector<Point> points;
  for (int i = 0; i < totalIntervals; ++i) {
    FrenetPoint fnPt = {sTraj.Eval(i * intervalLength),
                        dTraj.Eval(i * intervalLength)};
    points.push_back(map.FromFrenet(fnPt));
  }

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
    const double distance =
        Distance(points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
    const double v = distance / intervalLength;

    if (i > 1) {
      const Point v1{points[i].x - points[i - 1].x,
                     points[i].y - points[i - 1].y};
      const Point v2{points[i - 1].x - points[i - 2].x,
                     points[i - 1].y - points[i - 2].y};

      // This should be same as below.
      //      double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (prevS * distance);
      //      double turnRad = distance / sqrt(2 * (1 - cosPhi));
      //      double normalAcc = v * v / turnRad;

      const double cosPhi = GetCosAngle(v1, v2);
      const double sinPhi = sqrt(1 - cosPhi * cosPhi);
      const Point v3{points[i].x - points[i - 2].x,
                     points[i].y - points[i - 2].y};
      const double curve = 2 * sinPhi / Distance(0, 0, v3.x, v3.y);

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

        const double avgSpeed = avgSpeedSum / avgSpeedCount;

        if (hasPrevAvgSpeed) {
          const double avgCurve = avgCurveSum / avgSpeedCount;
          const double normalAcc = avgSpeed * avgSpeed * avgCurve;
          const double tangAcc =
              (avgSpeed - prevAvgSpeed) / (avgSpeedCount * intervalLength);
          const double fullAcc =
              sqrt(normalAcc * normalAcc + tangAcc * tangAcc);
          maxAccSoFar = std::max(maxAccSoFar, fullAcc);
        }

        prevAvgSpeed = avgSpeed;
        hasPrevAvgSpeed = true;

        maxSpeedSoFar = std::max(maxSpeedSoFar, avgSpeed);

        avgSpeedCount = 1;
        avgSpeedSum = speeds[i];
        avgCurveSum = curves[i];
      }
    }

    prevV = v;
    prevS = distance;
  }

  return {maxAccSoFar, maxSpeedSoFar};
}

double PowerLimit(double x, double limit, double threshold) {
  if (x > limit) {
    return 100.0;
  }

  if (x < threshold) {
    return 0.0;
  }

  const double res = (x - threshold) / (limit - threshold);
  return std::pow(res, 5);
}

double CartesianAccelerationLimitCost(const PolyFunction& sTraj,
                                      const PolyFunction& dTraj,
                                      double targetTime, double accLimit,
                                      double speedLimit, const Map& map) {
  const auto result =
      GetMaxCartesianAccelerationAndSpeed(sTraj, dTraj, targetTime, map);
  const double maxAccSoFar = result.first;
  const double maxSpeedSoFar = result.second;

  const double accCost = PowerLimit(maxAccSoFar, accLimit, 0.9 * accLimit);
  const double speedCost =
      PowerLimit(maxSpeedSoFar, speedLimit, 0.9 * speedLimit);

  return (10 * accCost + speedCost) / 11;
}

struct LaneDistanceConfig {
  int laneIdx;
  bool checkFront;
  bool checkBack;
};

double ExceedsSafeDistance(const PolyFunction& sTraj, const PolyFunction& dTraj,
                           double targetTime, double worldTimeOffset,
                           World& world, const Map& map,
                           const std::vector<LaneDistanceConfig>& lanesToCheck,
                           bool checkFrontOnly = true,
                           double minDistanceStrict = -1,
                           bool verbose = false) {
  const double intervalLength = kRefreshPeriodSeconds;
  const int totalIntervals = targetTime / intervalLength;

  if (lanesToCheck.empty()) {
    throw std::runtime_error("WHAT! must have at least one lane to check");
  }

  double maxCost = 0;

  for (int i = 0; i < totalIntervals; ++i) {
    const double currentTime = i * intervalLength;
    const FrenetPoint ourPosFn = {sTraj.Eval(currentTime),
                                  dTraj.Eval(currentTime)};
    const double ourSpeed = sTraj.Eval2(currentTime);
    const auto ourPos = map.FromFrenet(ourPosFn);

    const WorldSnapshot& snapshot =
        world.Simulate(currentTime + worldTimeOffset);

    for (const LaneDistanceConfig& cfg : lanesToCheck) {
      const double frontDistanceToKeep = GetMinDistanceToKeep(ourSpeed);

      // lanesToCheck can have index that is not in the snapshot
      const auto& carsRange =
          snapshot.GetAllCarsByLane().equal_range(cfg.laneIdx);

      for (auto it = carsRange.first; it != carsRange.second; ++it) {
        const auto carId = it->second;
        const auto& otherCar = snapshot.GetCarById(carId);

        double distanceToKeep = -1;

        if (minDistanceStrict >= 0) {
          // If the caller wants specific limit, use that.
          distanceToKeep = minDistanceStrict;
        } else {
          if (otherCar.fnPos.s < ourPosFn.s) {
            if (cfg.checkBack) {
              distanceToKeep = GetMinDistanceToKeep(otherCar.speed);
            }
          } else {
            if (cfg.checkFront) {
              distanceToKeep = frontDistanceToKeep;
            }
          }
        }

        if (distanceToKeep > 0) {
          const auto otherCarPos = map.FromFrenet(otherCar.fnPos);

          // Calculate distnace in cartesian, should probably be more precise on
          // turns
          const double distance =
              Distance(ourPos.x, ourPos.y, otherCarPos.x, otherCarPos.y);

          double cost = 0;
          if (distance <= distanceToKeep) {
            cost = std::abs(distanceToKeep - distance) / distanceToKeep;
          }
          maxCost = std::max(maxCost, cost);
        }  // if (distanceToKeep > 0)
      }    // for cars in lane
    }      // for lanes
  }        // for time

  return maxCost;
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
  const double step = kRefreshPeriodSeconds;
  const size_t totalIntervals = targetTime / step;

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

double ClosenessToCenterOfTheLane(const PolyFunction& sTraj,
                                  const PolyFunction& dTraj, double targetTime,
                                  double currentLaneD) {
  const double step = kRefreshPeriodSeconds;
  const size_t totalIntervals = targetTime / step;

  double maxDistance = 0;
  for (int i = 1; i < totalIntervals; ++i) {
    double d = dTraj.Eval(i * kRefreshPeriodSeconds);
    double dist = std::abs(d - currentLaneD);
    maxDistance = std::max(maxDistance, dist);
  }

  return ClosenessCost(maxDistance, 0, 1.0);
}

////////////////////////////////////////////////////////////////////////////////

Decider::Decider(double laneWidth, double minTrajectoryTimeSeconds,
                 double latencySeconds, const Map& map)
    : m_laneWidth(laneWidth),
      m_minTrajectoryTimeSeconds(minTrajectoryTimeSeconds),
      m_latencySeconds(latencySeconds),
      m_map(map),
      m_mode(Mode::kKeepingLane),
      m_followingCarId(-1),
      m_currentLane(-1),
      m_targetLane(-1),
      m_targetSpeed(0),
      m_updateNumber(0) {}

double Decider::GetSafeLaneOffset() const {
  return m_laneWidth / 2 - kMinToLaneBorderMeters;
}

std::pair<double, double> Decider::GetSafeLaneOffsets(int laneIdx) const {
  double laneD = CurrentLaneToDPos(laneIdx, m_laneWidth);
  double safeLaneOffset = GetSafeLaneOffset();
  return {laneD - safeLaneOffset, laneD + safeLaneOffset};
}

double Decider::LimitAccelerationAndSpeed(const PolyFunction& sTraj,
                                          const PolyFunction& dTraj,
                                          double targetTime) const {
  return CartesianAccelerationLimitCost(
      sTraj, dTraj, targetTime, kMaxAccelerationMs2, kMaxSpeedMs, m_map);
}

BestTrajectories Decider::BuildChangingLaneTrajectory(const State2D& startState,
                                                      int sourceLane,
                                                      int targetLane,
                                                      double targetSpeed,
                                                      World& world) {
  const double targetLaneD = CurrentLaneToDPos(targetLane, m_laneWidth);

  const auto sourceLaneOffsets = GetSafeLaneOffsets(sourceLane);
  const auto targetLaneOffsets = GetSafeLaneOffsets(targetLane);

  const double leftBoundary =
      std::min(sourceLaneOffsets.first, targetLaneOffsets.first);
  const double rightBoundary =
      std::max(sourceLaneOffsets.second, targetLaneOffsets.second);

  ConstantSpeedTarget target(targetSpeed, startState.s.s,
                             State{targetLaneD, 0, 0}, 0, 0);

  const auto reactionTimePenalty = [](const PolyFunction& sTraj,
                                      const PolyFunction& dTraj,
                                      double targetTime) { return targetTime; };

  std::cout << "should stay in between [" << leftBoundary << ", "
            << rightBoundary << "]\n";

  const auto stayInLanesPenalty = [leftBoundary, rightBoundary](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {
    return OutsideOfTheRoadPenalty(sTraj, dTraj, leftBoundary, rightBoundary,
                                   targetTime);
  };

  const auto hasGoodDistanceToOthers = [this, &world, sourceLane, targetLane](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {

    std::vector<LaneDistanceConfig> laneConfigs{
        LaneDistanceConfig{targetLane, true, true},
        LaneDistanceConfig{sourceLane, true, false}};

    return ExceedsSafeDistance(sTraj, dTraj, targetTime, m_latencySeconds,
                               world, m_map, laneConfigs);
  };

  const WeightedFunctions weighted{
      {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
      {1, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      {5, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
      {0.5, reactionTimePenalty},
      {400, stayInLanesPenalty},
      {500, hasGoodDistanceToOthers},
  };

  GenConfig cfg;
  cfg.sigmaS.s = kSigmaSS;
  cfg.sigmaS.v = kSigmaSV;
  cfg.sigmaS.acc = kSigmaSAcc;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 20;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  // This is not going to limit max lane change time, as we replan, but it works
  // quite well.
  cfg.maxTime = m_minTrajectoryTimeSeconds;
  cfg.timeStep = 0.2;

  const auto result = FindBestTrajectories(startState, target, cfg, weighted);
  return result;
}

BestTrajectories Decider::BuildKeepDistanceTrajectory(
    const State2D& startState, int followingCarId, double distanceToKeep,
    const WorldSnapshot& snapshot, World& world) {
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  const OtherCar otherCar = snapshot.GetCarById(followingCarId);

  // TODO: should actually use world to do such simulation
  ConstantSpeedTarget target(
      otherCar.speed, otherCar.fnPos.s, State{currentLaneD, 0, 0},
      distanceToKeep,
      0);  // no need for latency, as it was incorporated by snapshot

  const auto hasGoodDistanceToOthers = [this, &world](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {

    std::vector<LaneDistanceConfig> laneConfigs{
        LaneDistanceConfig{m_currentLane, true, false}};

    return ExceedsSafeDistance(sTraj, dTraj, targetTime, m_latencySeconds,
                               world, m_map, laneConfigs);
  };

  const auto safeOffsets = GetSafeLaneOffsets(m_currentLane);

  const WeightedFunctions weighted{
      {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
      {1, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      {1, std::bind(ClosenessToCenterOfTheLane, _1, _2, _3, currentLaneD)},
      {5, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
      {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, safeOffsets.first,
                      safeOffsets.second, _3)},
      {500, std::bind(hasGoodDistanceToOthers, _1, _2, _3)},
  };

  GenConfig cfg;
  cfg.sigmaS.s = kSigmaSS;
  cfg.sigmaS.v = kSigmaSV;
  cfg.sigmaS.acc = kSigmaSAcc;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 10;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  cfg.maxTime = 10 * m_minTrajectoryTimeSeconds;
  cfg.timeStep = 0.5;

  return FindBestTrajectories(startState, target, cfg, weighted);
}

BestTrajectories Decider::BuildKeepSpeedTrajectory(const State2D& startState,
                                                   double targetSpeed,
                                                   World& world) {
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  // TODO the s does not matter here.
  ConstantSpeedTarget target(
      targetSpeed, startState.s.s, State{currentLaneD, 0, 0}, 0,
      0);  // no need for latency, as it was incorporated by snapshot

  const auto safeOffsets = GetSafeLaneOffsets(m_currentLane);

  const auto closenessToTargetSpeed = [targetSpeed](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {
    double sumSpeed = 0;
    int count = 0;
    for (int i = 1; i < targetTime / kRefreshPeriodSeconds; ++i) {
      sumSpeed += sTraj.Eval2(i * kRefreshPeriodSeconds);
      ++count;
    }
    return ClosenessCost(sumSpeed / count, targetSpeed, targetSpeed);
  };

  const auto hasGoodDistanceToOthers = [this, &world](
      const PolyFunction& sTraj, const PolyFunction& dTraj, double targetTime) {

    std::vector<LaneDistanceConfig> laneConfigs{
        LaneDistanceConfig{m_currentLane, true, false}};

    return ExceedsSafeDistance(sTraj, dTraj, targetTime, m_latencySeconds,
                               world, m_map, laneConfigs);
  };

  const WeightedFunctions weighted{
      {1, closenessToTargetSpeed},
      {1, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
      {1, std::bind(ClosenessToCenterOfTheLane, _1, _2, _3, currentLaneD)},
      {5, std::bind(&Decider::LimitAccelerationAndSpeed, this, _1, _2, _3)},
      {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, safeOffsets.first,
                      safeOffsets.second, _3)},
      {500, std::bind(hasGoodDistanceToOthers, _1, _2, _3)}

  };

  GenConfig cfg;
  cfg.sigmaS.s = 0;
  cfg.sigmaS.v = 2;
  cfg.sigmaS.acc = kSigmaSAcc;
  cfg.sigmaD.s = kSigmaDS;
  cfg.sigmaD.v = kSigmaDV;
  cfg.sigmaD.acc = kSigmaDAcc;
  cfg.samplesCount = 10;
  cfg.minTime = m_minTrajectoryTimeSeconds;
  cfg.maxTime = 5 * m_minTrajectoryTimeSeconds;
  cfg.timeStep = 0.5;

  const auto result =
      FindBestTrajectories(startState, target, cfg, weighted, true);

  // const auto r = GetMaxCartesianAccelerationAndSpeed(result.s, result.d,
  //                                                    result.time, m_map);
  // std::cout << "maxAcc=" << r.first << ", maxSpeed=" << r.second << "\n";
  return result;
}

std::pair<bool, BestTrajectories> Decider::HandleKeepSpeedState(
    const State2D& startState, const WorldSnapshot& snapshot, World& world,
    const LaneToOccupancy& laneOccupancy) {
  return {false, {}};
}

std::pair<bool, BestTrajectories> Decider::HandleChangingLaneState(
    const State2D& startState, const WorldSnapshot& snapshot, World& world,
    const LaneToOccupancy& laneOccupancy) {
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);
  bool areWeThereYet =
      (m_currentLane == m_targetLane) &&
      (std::abs(currentLaneD - startState.d.s) < m_laneWidth / 10);

  const char* dest = m_changingLeft ? "left" : "right";

  if (!areWeThereYet) {
    // Adjust target speed in case a car in that lane is slowing down.
    const auto& laneSpeed = laneOccupancy.at(m_targetLane);
    double targetSpeed = m_targetSpeed;

    if (laneSpeed.first && laneSpeed.second.speed < m_targetSpeed) {
      targetSpeed = laneSpeed.second.speed;
    }

    m_targetSpeed = targetSpeed;

    std::cout << "Still changing lane to " << dest
              << ", targetSpeed=" << m_targetSpeed << "\n";

    return {true,
            BuildChangingLaneTrajectory(startState, m_sourceLane, m_targetLane,
                                        targetSpeed, world)};
  }

  std::cout << "Reached the target lane " << dest << "\n";

  return {false, {}};
}

BestTrajectories Decider::ChooseBestTrajectory(
    const State2D& startState, const std::vector<OtherCarSensor>& sensors) {
  ++m_updateNumber;

  m_currentLane = DPosToCurrentLane(startState.d.s, m_laneWidth);
  const double currentLaneD = CurrentLaneToDPos(m_currentLane, m_laneWidth);

  std::cout << "Current lane " << m_currentLane << "\n";

  World world(sensors, m_laneWidth);
  const auto& snapshot = world.Simulate(m_latencySeconds);

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

  DisplayCarsByLane(snapshot);
  DisplayLaneOccupancy(speeds);

  switch (m_mode) {
    case Mode::kChangingLane: {
      auto result =
          HandleChangingLaneState(startState, snapshot, world, speeds);
      if (result.first) {
        return result.second;
      }
    } break;
    case Mode::kKeepingLane: {
      auto result = HandleKeepSpeedState(startState, snapshot, world, speeds);
      if (result.first) {
        return result.second;
      }
    } break;
  }

  auto maxSpeedLane = std::max_element(
      begin(speeds), end(speeds), [](const LaneToOccupancy::value_type& p1,
                                     const LaneToOccupancy::value_type& p2) {
        return p1.second.second.speed < p2.second.second.speed;
      });

  std::cout << "Max speed lane " << maxSpeedLane->first << "\n";

  std::vector<BestTrajectories> candidates;
  candidates.push_back(
      BuildKeepSpeedTrajectory(startState, kTargetKeepSpeed, world));

  const auto& currentLane = speeds[m_currentLane];
  if (currentLane.first) {
    double distance = currentLane.second.fnPos.s - startState.s.s;
    std::cout << "Distance to vehicle in current lane " << distance
              << std::endl;
    if (distance < kOtherVehicleMonitorDistance) {
      double distanceToKeep = GetMinDistanceToKeep(currentLane.second.speed);
      candidates.push_back(BuildKeepDistanceTrajectory(
          startState, currentLane.second.id, distanceToKeep, snapshot, world));
    }

    if (maxSpeedLane->first != m_currentLane) {
      // If max speed lane is empty, use our curent speed as a target,otherwise
      // adjust to the speed of the lane.
      double targetSpeed = startState.s.v;
      if (maxSpeedLane->second.first) {
        targetSpeed = maxSpeedLane->second.second.speed;
      }

      auto trajectory = BuildChangingLaneTrajectory(
          startState, m_currentLane, maxSpeedLane->first, targetSpeed, world);

      if (trajectory.cost < 200) {
        std::cout << "Changing lane from " << m_currentLane << " to "
                  << maxSpeedLane->first << ", targetSpeed=" << targetSpeed
                  << "\n";

        // Shortcut here
        m_mode = Mode::kChangingLane;
        m_changingLeft = maxSpeedLane->first < m_currentLane;
        m_sourceLane = m_currentLane;
        m_targetLane = maxSpeedLane->first;
        m_targetSpeed = targetSpeed;
        return trajectory;
      }

      std::cout << "Changing lanes not feasible\n";
    }
  }

  m_mode = Mode::kKeepingLane;

  const auto& minCostTrajIt = std::min_element(
      candidates.begin(), candidates.end(),
      [](const BestTrajectories& t1, const BestTrajectories& t2) -> bool {
        return t1.cost < t2.cost;
      });

  std::cout << "Best trajectory idx=" << (minCostTrajIt - candidates.begin())
            << std::endl;
  return *minCostTrajIt;
}

////////////////////////////////////////////////////////////////////////////////

Planner::Planner(const Map& map)
    : m_updatePeriod(kRefreshPeriodSeconds),
      m_laneWidth(kLaneWidthMeters),
      m_map(map),
      m_decider(kLaneWidthMeters, kMinTrajectoryTimeSeconds,
                kAlgorithmLatencySeconds, m_map),
      m_updateNumber(0) {}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCarSensor>& sensors) {
  if (m_updateNumber == 0) {
    m_prevUpdateTime = std::chrono::high_resolution_clock::now();
  }

  // Index of the next position to move to.
  const ssize_t nextPosIdx = m_plannedPath.size() - unprocessedPath.size();

  const double currentTrajectoryTime =
      (nextPosIdx > 0) ? (nextPosIdx - 1) * m_updatePeriod : 0;

  const double remainingTrajectoryTime =
      unprocessedPath.size() * m_updatePeriod;

  const bool isTimeToReplan =
      (m_updateNumber == 0) || (currentTrajectoryTime >= kReplanPeriodSeconds);

  // We can continue only if we had any points processed.
  const bool continueTrajectory = (nextPosIdx > 0) && !m_plannedPath.empty();

  // std::cout << "nextIdx=" << nextPosIdx << ", s=" << car.fp.s
  //           << ", d=" << car.fp.d << ", speed=" << car.car.speed
  //           << ", x=" << car.car.pos.x << ", y=" << car.car.pos.y
  //           << ", unprocessed_size=" << unprocessedPath.size()
  //           << (isTimeToReplan ? ", replanning" : ",") << std::endl;

  // if (continueTrajectory) {
  //   double expectedS = m_plannedPath[nextPosIdx - 1].fn.s.s;
  //   double expectedD = m_plannedPath[nextPosIdx - 1].fn.d.s;
  //   double expectedX = m_plannedPath[nextPosIdx - 1].pt.x;
  //   double expectedY = m_plannedPath[nextPosIdx - 1].pt.y;

  //   if (std::abs(expectedS - car.fp.s) > 0.5 ||
  //       (std::abs(expectedD - car.fp.d) > 0.5)) {
  //     std::cout << "FN CHECKPOINT FAILED! expected s=" << expectedS
  //               << ", d=" << expectedD << ", got s=" << car.fp.s
  //               << ", d=" << car.fp.d << std::endl;
  //   }

  //   if (std::abs(expectedX - car.car.pos.x) > 0.5 ||
  //       (std::abs(expectedY - car.car.pos.y) > 0.5)) {
  //     std::cout << "XY CHECKPOINT FAILED! expected x=" << expectedX
  //               << ", y=" << expectedY << ", got x=" << car.car.pos.x
  //               << ", y=" << car.car.pos.y << std::endl;
  //   }
  // }

  // Check whether trajectory is still valid.
  // If it is - keep going, until there is almost nothing left in the current
  // trajectory.

  if (!isTimeToReplan) {
    return unprocessedPath;
  }

  // Replan trajectories

  State2D startState{State{car.fp.s, car.car.speed, 0}, State{car.fp.d, 0, 0}};

  const size_t pointsToKeep =
      std::min<size_t>(kPointsToKeep, unprocessedPath.size());
  const ssize_t startPosIdx = nextPosIdx - 1 + pointsToKeep;

  // std::cout << "points to keep=" << pointsToKeep << "\n";

  if (continueTrajectory) {
    // When continueTrajectory is set, nextPosIdx > 0
    startState = m_plannedPath[startPosIdx].fn;

    std::cout << std::setfill('-') << std::setw(80) << '\n'
              << std::setfill(' ');
    std::cout << "Starting trajectory at " << startPosIdx << ", time offset "
              << startPosIdx * kRefreshPeriodSeconds << "\n";
    std::cout << "s=["
              << "s=" << startState.s.s << ", v=" << startState.s.v
              << ", acc=" << startState.s.acc << "]\n";
    std::cout << "d=["
              << "d=" << startState.d.s << ", v=" << startState.d.v
              << ", acc=" << startState.d.acc << "]\n";
  }

  using sec = std::chrono::duration<double>;
  const auto start = std::chrono::high_resolution_clock::now();
  const auto updatePrecision =
      std::chrono::duration_cast<sec>(start - m_prevUpdateTime).count() -
      currentTrajectoryTime;
  m_prevUpdateTime = start;

  const auto bestTrajectory =
      m_decider.ChooseBestTrajectory(startState, sensors);

  const auto deciderTime = std::chrono::duration_cast<sec>(
      std::chrono::high_resolution_clock::now() - start);

  std::cout << std::setfill('-') << std::setw(80) << '\n'
            << std::setfill(' ') << std::endl;

  std::cout << "Decider time=" << deciderTime.count()
            << ", update time precision=" << updatePrecision << "\n";

  if (bestTrajectory.cost > kHighCostCheckpoint) {
    std::cout << "COST CHECKPOINT FAILED: cost=" << bestTrajectory.cost
              << ", why would it choose such a trajectory?" << std::endl;
  }

  std::cout << std::setfill('-') << std::setw(80) << '\n'
            << std::setfill(' ') << std::endl;

  std::vector<FullState> planned;
  if (continueTrajectory &&
      (nextPosIdx + pointsToKeep - 1 < m_plannedPath.size())) {
    // append points from previous trajectory so that transition between
    // trajectories would be smooth
    planned.insert(planned.begin(), m_plannedPath.begin() + nextPosIdx,
                   m_plannedPath.begin() + nextPosIdx + pointsToKeep - 1);
    // std::cout << "Copied [" << nextPosIdx << ";"
    //           << nextPosIdx + pointsToKeep - 1 << ")\n";
  }

  for (int i = 0; i < bestTrajectory.time / m_updatePeriod; ++i) {
    const double t = i * m_updatePeriod;
    State2D fn;
    // Clamp s position so that next time we would start from the beginning of
    // the lap.
    fn.s.s = m_map.ClampFrenetS(bestTrajectory.s.Eval(t));
    fn.s.v = bestTrajectory.s.Eval2(t);
    fn.s.acc = bestTrajectory.s.Eval3(t);

    fn.d.s = bestTrajectory.d.Eval(t);
    fn.d.v = bestTrajectory.d.Eval2(t);
    fn.d.acc = bestTrajectory.d.Eval3(t);

    const Point pt = m_map.FromFrenet(FrenetPoint{fn.s.s, fn.d.s});
    planned.push_back(FullState{fn, pt});
  }

  // int i = 0;
  // std::cout << "i;s_pos;s_v;d_pos;d_v\n";
  // for (const auto& item : m_plannedPath) {
  //   std::cout << i << ";" << item.fn.s.s << ";" << item.fn.s.v << ";"
  //             << item.fn.d.s << ";" << item.fn.d.v << "\n";
  //   ++i;
  // }
  // std::cout << "\n";

  m_plannedPath = planned;
  m_updateNumber++;

  std::vector<Point> result;
  for (const auto& state : m_plannedPath) {
    result.push_back(state.pt);
  }
  return result;
}
