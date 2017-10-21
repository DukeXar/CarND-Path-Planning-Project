#include "planner.h"
#include <iostream>
#include <array>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include "Dense"
#include "map.h"
#include "utils.h"


int DPosToCurrentLane(double d, double laneWidth) {
  return d / laneWidth;
}

double CurrentLaneToDPos(int laneIdx, double laneWidth) {
  return laneIdx * laneWidth + laneWidth / 2;
}


class FixedTarget : public Target {
public:
  explicit FixedTarget(const State2D & state): m_state(state) {}
  virtual State2D At(double time) const override;

private:
  State2D m_state;
};

State2D FixedTarget::At(double time) const {
  return m_state;
}


class ConstantSpeedTarget : public Target {
public:
  explicit ConstantSpeedTarget(double speed, double startS, const State & stateD, double distance = 0, double latency = 0)
  : m_speed(speed), m_startS(startS), m_stateD(stateD), m_distance(distance), m_latency(latency) {}
  
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


class World {
public:
  World(const std::vector<OtherCar> & sensors, double laneWidth);
  
  bool GetClosestCar(int laneIdx, double s, OtherCar * result) const;
  OtherCar GetCarById(int id) const;
  
private:
  double m_laneWidth;
  std::unordered_map<int, std::vector<OtherCar>> m_cars;
  std::unordered_map<int, OtherCar> m_byId;
};

World::World(const std::vector<OtherCar> & sensors, double laneWidth): m_laneWidth(laneWidth) {
  for (const auto & car : sensors) {
    int lane = DPosToCurrentLane(car.fnPos.d, m_laneWidth);
    if (lane > 0) {
      m_cars[lane].push_back(car);
      m_byId[car.id] = car;
//      std::cout << "lane=" << lane << ", d=" << car.fnPos.d << ", s=" << car.fnPos.s << std::endl;
    }
  }
  
  for (auto & laneAndCars : m_cars) {
    std::sort(laneAndCars.second.begin(), laneAndCars.second.end(), [](const OtherCar & c1, const OtherCar & c2) {
      return c1.fnPos.s < c2.fnPos.s;
    });
  }

//  std::cout << "Cars by lane: \n";
//  for (const auto & laneAndCars : m_cars) {
//    std::cout << laneAndCars.first << "=[";
//    for (const auto & car : laneAndCars.second) {
//      double v = std::sqrt(car.speed.x * car.speed.x + car.speed.y * car.speed.y);
//      std::cout << car.id << "(s=" << car.fnPos.s << ", v=" << v << ")" << ",";
//    }
//    std::cout << "]\n";
//  }
//  std::cout << std::endl;
}

bool World::GetClosestCar(int laneIdx, double s, OtherCar * result) const {
  auto pos = m_cars.find(laneIdx);
  if (pos == m_cars.end()) {
    return false;
  }
  
  const auto & carsInLane = pos->second;
  
  const OtherCar example{0, {0, 0}, {0, 0}, {s, 0}};
  auto lb = std::lower_bound(carsInLane.begin(), carsInLane.end(), example,
                             [](const OtherCar & c1, const OtherCar & c2) {
    return c1.fnPos.s < c2.fnPos.s;
  });
  
  if (lb == carsInLane.end()) {
    return false;
  }
  
  if (result) {
    *result = *lb;
  }

  return true;
}

OtherCar World::GetCarById(int id) const {
  auto pos = m_byId.find(id);
  if (pos == m_byId.end()) {
    throw std::runtime_error("Not found");
  }
  return pos->second;
}

////////////////////////////////////////////////////////////////////////////////////////////////////


typedef std::vector<std::pair<double, CostFunction>> WeightedFunctions;

double WeightedCostFunction(const WeightedFunctions & weigtedFunctions,
                            const PolyFunction & sTraj, const PolyFunction & dTraj,
                            double targetTime) {
  double result = 0;
  for (const auto & wf : weigtedFunctions) {
    result += wf.first * wf.second(sTraj, dTraj, targetTime);
  }
  return result;
}

double AllGoodFunction(const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
  return 1.0;
}

double Logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double ClosenessCost(double x1, double x2, double sigma) {
  return Logistic(std::abs(x1 - x2) / sigma);
}

double SpeedLimitCost(const PolyFunction & sTraj, const PolyFunction & dTraj,
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

double AccelerationLimitCost(const PolyFunction & sTraj, const PolyFunction & dTraj,
                             double targetTime, double accLimit) {
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
  
  double a = (1-percent) * accLimit;
  double k = accLimit / a;
  return k * maxAccSoFar - accLimit;
}

std::pair<double, double> GetMaxCartesianAccelerationAndSpeed(const PolyFunction & sTraj, const PolyFunction & dTraj,
                                                              double targetTime, const Map & map) {
  double maxAccSoFar = 0;
  double maxSpeedSoFar = 0;

  // TODO: This is so slow
  //size_t totalIntervals = 100;
  //double intervalLength = targetTime / totalIntervals;
  double intervalLength = 0.02;
  int totalIntervals = targetTime / intervalLength;
  
  std::vector<FrenetPoint> fnPoints;
  for (int i = 0; i < totalIntervals; ++i) {
    fnPoints.push_back({sTraj.Eval(i*intervalLength), dTraj.Eval(i*intervalLength)});
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
    double distance = Distance(points[i-1].x, points[i-1].y, points[i].x, points[i].y);
    double v = distance / intervalLength;
    
    if (i > 1) {
      Point v1{points[i].x - points[i-1].x, points[i].y - points[i-1].y};
      Point v2{points[i-1].x - points[i-2].x, points[i-1].y - points[i-2].y};
      
      // This should be same as below.
      //      double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (prevS * distance);
      //      double turnRad = distance / sqrt(2 * (1 - cosPhi));
      //      double normalAcc = v * v / turnRad;
      
      double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (Distance(0, 0, v2.x, v2.y) * Distance(0, 0, v1.x, v1.y));
      Point v3{points[i].x - points[i-2].x, points[i].y - points[i-2].y};
      double curve = 2 * sqrt(1 - cosPhi * cosPhi) / Distance(0, 0, v3.x, v3.y);
      
      maxAccS = std::max(maxAccS, sTraj.Eval3((i-1) * intervalLength));

      speeds[i] = v;
      curves[i] = curve;
      avgSpeedSum += v;
      avgCurveSum += curve;
      ++avgSpeedCount;

      // Average same way as simulator does, otherwise it gets inconsistent.
      if (avgSpeedCount > 10) {
        --avgSpeedCount;
        avgSpeedSum -= speeds[i-avgSpeedCount];
        avgCurveSum -= curves[i-avgSpeedCount];

        double avgSpeed = avgSpeedSum / avgSpeedCount;

        if (hasPrevAvgSpeed) {
          double avgCurve = avgCurveSum / avgSpeedCount;
          double normalAcc = avgSpeed * avgSpeed * avgCurve;
          double tangAcc = (avgSpeed - prevAvgSpeed) / (avgSpeedCount * intervalLength);
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


double CartesianAccelerationLimitCost(const PolyFunction & sTraj, const PolyFunction & dTraj,
                                      double targetTime, double accLimit, double speedLimit, const Map & map) {

  auto result = GetMaxCartesianAccelerationAndSpeed(sTraj, dTraj, targetTime, map);
  double maxAccSoFar = result.first;
  double maxSpeedSoFar = result.second;
  
  double accCost = PowerLimit(maxAccSoFar, accLimit, 0.9 * accLimit);
  double speedCost = PowerLimit(maxSpeedSoFar, speedLimit, 0.9 * speedLimit);
  
  return (10 * accCost + speedCost) / 11;
}

double ClosenessToTargetSState(const PolyFunction & sTraj, const PolyFunction & dTraj,
                               const Target & target, double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;

  cost += ClosenessCost(targetState.s.s, sTraj.Eval(targetTime), kSigmaSS);
  cost += ClosenessCost(targetState.s.v, sTraj.Eval2(targetTime), kSigmaSV);
  cost += ClosenessCost(targetState.s.acc, sTraj.Eval3(targetTime), kSigmaSAcc);
  
  return cost;
}

double ClosenessToTargetDState(const PolyFunction & sTraj, const PolyFunction & dTraj,
                               const Target & target, double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;
  
  cost += ClosenessCost(targetState.d.s, dTraj.Eval(targetTime), kSigmaDS);
  cost += ClosenessCost(targetState.d.v, dTraj.Eval2(targetTime), kSigmaDV);
  cost += ClosenessCost(targetState.d.acc, dTraj.Eval3(targetTime), kSigmaDAcc);
  
  return cost;
}

double OutsideOfTheRoadPenalty(const PolyFunction & sTraj, const PolyFunction & dTraj,
                               double roadLeft, double roadRight, double targetTime) {
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

//class StateInterface {
//public:
//  virtual std::pair<PolyFunction, PolyFunction> ChooseTrajectory(const State2D & startState,
//                                                                 const std::vector<OtherCar> & sensors);
//
//};

const int kKeepSpeedState = 0;
const int kFollowVehicleState = 1;
const int kChangingLaneLeftState = 2;
const int kChangingLaneRightState = 3;

Decider::Decider(double horizonSeconds, double laneWidth, double minTrajectoryTimeSeconds, double latencySeconds, const Map & map)
: m_horizonSeconds(horizonSeconds), m_laneWidth(laneWidth), m_minTrajectoryTimeSeconds(minTrajectoryTimeSeconds),
m_latencySeconds(latencySeconds), m_map(map), m_state(kKeepSpeedState),
m_followingCarId(-1), m_targetLane(-1), m_targetSpeed(0), m_updateNumber(0) {
}

BestTrajectories Decider::ChooseBestTrajectory(const State2D & startState, const std::vector<OtherCar> & sensors) {
  // README: Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.
  const double kMaxAccelerationMs2 = 10;
  const double kMaxSpeedMs = MiphToMs(50);
  
  ++m_updateNumber;
  
  const auto outsideOfTheRoadPenalty = [this](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    return OutsideOfTheRoadPenalty(sTraj, dTraj, 0 + m_laneWidth / 8, 3 * m_laneWidth - m_laneWidth / 8, targetTime);
  };
  
  const auto cartesianAccelerationAndSpeedLimit = [this, kMaxAccelerationMs2, kMaxSpeedMs](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    return CartesianAccelerationLimitCost(sTraj, dTraj, targetTime, kMaxAccelerationMs2, kMaxSpeedMs, m_map);
  };

  auto reactionTimePenalty = [](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    return targetTime;
  };

  World world(sensors, m_laneWidth);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  const int kCurrentLaneIdx = DPosToCurrentLane(startState.d.s, m_laneWidth);
  const double kCurrentLaneD = CurrentLaneToDPos(kCurrentLaneIdx, m_laneWidth);
  const double kOtherVehicleFollowDistance = 30; // TODO: should not it be possible to follow a vehicle from far?
  
  if (m_targetLane == -1) {
    m_targetLane = kCurrentLaneIdx;
  }

  switch (m_state) {
    case kKeepSpeedState: {
      OtherCar closestCar;
      if (world.GetClosestCar(kCurrentLaneIdx, startState.s.s, &closestCar)) {
        double distance = closestCar.fnPos.s - startState.s.s;
        
        // TODO: this should check the car in future
        std::cout << "Found closest car id=" << closestCar.id << ", s=" << closestCar.fnPos.s << ", my s=" << startState.s.s << std::endl;
        if (distance <= kOtherVehicleFollowDistance) {
          m_state = kFollowVehicleState;
          m_followingCarId = closestCar.id;
        }
      }
      // This is to simulate lane change
//      if (m_updateNumber == 9) {
//        m_targetLane = m_targetLane + 1;
//        m_state = kChangingLaneRightState;
//        m_targetSpeed = startState.s.v;
//      }
    } break;
    case kFollowVehicleState: {
      OtherCar closestCar;
      if (world.GetClosestCar(kCurrentLaneIdx, startState.s.s, &closestCar)) {
        double distance = closestCar.fnPos.s - startState.s.s;

        if (distance > kOtherVehicleFollowDistance) {
          std::cout << "No cars to follow" << ", my s=" << startState.s.s << std::endl;
          m_state = kKeepSpeedState;
          m_followingCarId = -1;
        } else {
          if (m_followingCarId != closestCar.id) {
            std::cout << "Found new closest car id=" << closestCar.id << ", s=" << closestCar.fnPos.s << ", my s=" << startState.s.s << std::endl;
            m_followingCarId = closestCar.id;
          }
        }
      }
    } break;
    case kChangingLaneRightState: {
      if (kCurrentLaneIdx == m_targetLane && (std::abs(kCurrentLaneD - startState.d.s) < m_laneWidth / 8)) {
        std::cout << "Reached the target (right) lane" << std::endl;
        m_state = kKeepSpeedState;
      }
    } break;
    case kChangingLaneLeftState: {
      if (kCurrentLaneIdx == m_targetLane) {
        std::cout << "Reached the target (left) lane" << std::endl;
        m_state = kKeepSpeedState;
      }
    } break;
    default:
      throw std::runtime_error("Not implemented");
      break;
  };
  
  switch (m_state) {
    case kKeepSpeedState: {
      std::cout << "Keeping speed" << std::endl;

      const double kTargetSpeed = MiphToMs(48);

      ConstantSpeedTarget target(kTargetSpeed, startState.s.s, State{kCurrentLaneD, 0, 0}, 0, 0);

      const double laneLeft = kCurrentLaneD - m_laneWidth/8;
      const double laneRight = kCurrentLaneD + m_laneWidth/8;

      WeightedFunctions weighted{
        {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
        {20, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
        {1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
        {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, laneLeft, laneRight, _3)},
        {50, std::bind(cartesianAccelerationAndSpeedLimit, _1, _2, _3)}
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
      cfg.maxTime = m_horizonSeconds + 2;
      cfg.timeStep = 0.2;

      return FindBestTrajectories(startState, target, cfg, costFunction);
    } break;
    
    case kFollowVehicleState: {
      OtherCar otherCar = world.GetCarById(m_followingCarId);
      double otherCarSpeedModulo = std::sqrt(otherCar.speed.x * otherCar.speed.x + otherCar.speed.y * otherCar.speed.y);
      
      // TODO: this should be based on the target vehicle speed, maximum allowed acceleration to go to full stop
      // s = 0.5/a * v^2
      // lets do 2x
      double distanceToKeep = 2 * (otherCarSpeedModulo * otherCarSpeedModulo) * 0.5 / kMaxAccelerationMs2;

      std::cout << "Following vehicle id=" << m_followingCarId << ", distance=" << distanceToKeep << std::endl;
      
      ConstantSpeedTarget target(otherCarSpeedModulo,
                                 otherCar.fnPos.s,
                                 State{kCurrentLaneD, 0, 0},
                                 distanceToKeep,
                                 m_latencySeconds);
      
      const double laneLeft = kCurrentLaneD - m_laneWidth/2 + m_laneWidth * 1/8;
      const double laneRight = kCurrentLaneD + m_laneWidth/2 - m_laneWidth * 1/8;
      WeightedFunctions weighted{
        {30, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
        {1, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
        //{90, std::bind(speedLimit, _1, _2, _3)},
        //{200, std::bind(accelerationLimit, _1, _2, _3)},
        {1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
        {300, std::bind(OutsideOfTheRoadPenalty, _1, _2, laneLeft, laneRight, _3)},
        //{5, reactionTimePenalty},
        {300, std::bind(cartesianAccelerationAndSpeedLimit, _1, _2, _3)}
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
      cfg.maxTime = m_horizonSeconds + 10;
      cfg.timeStep = 0.2;
      
      auto result = FindBestTrajectories(startState, target, cfg, costFunction);
      std::cout << "Future state: car pos=" << target.At(result.time).s.s << ", car speed="
         << otherCarSpeedModulo << ", my pos=" << result.s.Eval(result.time)
         << ", my speed=" << result.s.Eval2(result.time)
         << std::endl;
      return result;
    } break;
    
    case kChangingLaneLeftState: {
      
    } break;
    
    case kChangingLaneRightState: {
      std::cout << "Changing lane right, targetSpeed=" << m_targetSpeed << std::endl;
      
      double targetLaneD = CurrentLaneToDPos(m_targetLane, m_laneWidth);

      ConstantSpeedTarget target(m_targetSpeed, startState.s.s, State{targetLaneD, 0, 0}, 0, m_latencySeconds);
      
      const double laneLeft = kCurrentLaneD - m_laneWidth / 2 + m_laneWidth / 8;
      const double laneRight = targetLaneD + m_laneWidth / 2 - m_laneWidth / 8;
      
      WeightedFunctions weighted{
        {1, std::bind(ClosenessToTargetSState, _1, _2, target, _3)},
        {10, std::bind(ClosenessToTargetDState, _1, _2, target, _3)},
        //{1000, std::bind(outsideOfTheRoadPenalty, _1, _2, _3)},
        {500, std::bind(OutsideOfTheRoadPenalty, _1, _2, laneLeft, laneRight, _3)},
        {20, std::bind(cartesianAccelerationAndSpeedLimit, _1, _2, _3)},
        {1, reactionTimePenalty}
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
      cfg.maxTime = m_horizonSeconds + 10;
      cfg.timeStep = 0.2;
      
      m_state = kChangingLaneRightState;

      return FindBestTrajectories(startState, target, cfg, costFunction);
    } break;
    
    default:
      throw std::runtime_error("Not implemented");
      break;
  }
  
  throw std::runtime_error("Must not reach");
}


////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {
  const double kHorizonSeconds = 5; // ???
  const double kReplanPeriodSeconds = 0.7;
  const double kAlgorithmLatencySeconds = 0.4;
  const int kPointsToKeep = kAlgorithmLatencySeconds / 0.02; // 1 second latency of the algorithm, 50 points
  const double kMaxUpdateLatencySeconds = 2;
  const double kMinTrajectoryTimeSeconds = std::max(kReplanPeriodSeconds, kMaxUpdateLatencySeconds);
} // namespace

Planner::Planner(const Map& map, double updatePeriodSeconds, double laneWidthMeters)
    : m_updatePeriod(updatePeriodSeconds),
      m_laneWidth(laneWidthMeters),
      m_map(map),
      m_decider(kHorizonSeconds, laneWidthMeters, kMinTrajectoryTimeSeconds,
                kAlgorithmLatencySeconds, m_map),
      m_trajectoryOffsetIdx(0),
      m_hasTrajectory(false),
      m_updateNumber(0)
{}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCar>& sensors) {
  
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
  
//  std::cout << "idx=" << currentPosIdx << ", s=" << car.fp.s << ", d=" << car.fp.d << ", speed=" << car.car.speed << ", phi=" << car.car.yaw << (isTimeToReplan ? ", replanning" : ",") << std::endl;
  
  std::cout << "idx=" << currentPosIdx << ", s=" << car.fp.s << ", speed=" << car.car.speed << ", up=" << unprocessedPath.size()
    << (isTimeToReplan ? ", replanning" : ",") << std::endl;
  
  if (!isTimeToReplan) {
    return unprocessedPath;
  }

  // Replan trajectories
  
  State2D startState{State{car.fp.s, car.car.speed, 0}, State{car.fp.d, 0, 0}};

  if (m_hasTrajectory && continueTrajectory) {
    ssize_t nextPosIdx = currentPosIdx + kPointsToKeep - m_trajectoryOffsetIdx;

    startState.s.s = m_plannedTrajectories.s.Eval(nextPosIdx * m_updatePeriod);
    startState.s.v = m_plannedTrajectories.s.Eval2(nextPosIdx * m_updatePeriod);
    startState.s.acc = m_plannedTrajectories.s.Eval3(nextPosIdx * m_updatePeriod);

    startState.d.s = m_plannedTrajectories.d.Eval(nextPosIdx * m_updatePeriod);
    startState.d.v = m_plannedTrajectories.d.Eval2(nextPosIdx * m_updatePeriod);
    startState.d.acc = m_plannedTrajectories.d.Eval3(nextPosIdx * m_updatePeriod);
    
    std::cout << "Start ";
    std::cout << "s=[" << "s=" << startState.s.s << ", v=" << startState.s.v << ", acc=" << startState.s.acc << "], ";
    std::cout << "time=" << nextPosIdx * m_updatePeriod << std::endl;
  }

  auto start = std::chrono::high_resolution_clock::now();
  auto bestTrajectory = m_decider.ChooseBestTrajectory(startState, sensors);
  using sec = std::chrono::duration<double>;
  auto delta = std::chrono::duration_cast<sec>(std::chrono::high_resolution_clock::now() - start);
  std::cout << "Decider time=" << delta.count() << ", update time delta=" << (std::chrono::duration_cast<sec>(start - m_prevUpdateTime).count() - currentPosIdx * m_updatePeriod) << std::endl;
  m_prevUpdateTime = start;

  std::vector<Point> planned;
  if (continueTrajectory && (currentPosIdx + kPointsToKeep - 1 < m_plannedPath.size())) {
    // append points from previous trajectory so that transition between trajectories would be smooth
    planned.insert(planned.begin(), m_plannedPath.begin() + currentPosIdx, m_plannedPath.begin() + currentPosIdx + kPointsToKeep);
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
//    double dist = Distance(planned[i-1].x, planned[i-1].y, planned[i].x, planned[i].y);
//    //std::cout << plannedS[i] << "\t" << plannedS[i] - plannedS[i-1] << "\t" << dist/m_updatePeriod << "\n";
//    Point v1{planned[i].x - planned[i-1].x, planned[i].y - planned[i-1].y};
//    Point v2{planned[i-1].x - planned[i-2].x, planned[i-1].y - planned[i-2].y};
//
//    double cosPhi = (v1.x * v2.x + v1.y * v2.y) / (Distance(0, 0, v2.x, v2.y) * Distance(0, 0, v1.x, v1.y));
//    double phi = std::acos(cosPhi);
//    //double turnRad = prevS / sqrt(2 * (1 - cosPhi));
//    //double normalAcc = prevV * prevV / turnRad;
//    std::cout << "i=" << i << "\t" << rad2deg(phi) << ", x=" << planned[i].x << ", y=" << planned[i].y << std::endl;
//  }
//  std::cout << std::endl;

  m_plannedPath = planned;
  m_plannedTrajectories = bestTrajectory;
  m_hasTrajectory = true;
  m_updateNumber++;
  return m_plannedPath;
}
