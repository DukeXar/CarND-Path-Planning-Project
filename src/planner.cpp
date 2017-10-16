#include "planner.h"
#include <iostream>
#include <array>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include "Dense"
#include "map.h"
#include "utils.h"


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
  explicit ConstantSpeedTarget(double speed, double startS, const State & stateD)
  : m_speed(speed), m_startS(startS), m_stateD(stateD) {}
  
  virtual State2D At(double time) const override;
  
private:
  double m_speed;
  double m_startS;
  State m_stateD;
};

State2D ConstantSpeedTarget::At(double time) const {
  if (time < 0) {
    throw std::runtime_error("WHOA!");
  }
  double s = m_speed * time + m_startS;
  return State2D{State{s, m_speed, 0}, m_stateD};
}


class World {
public:
};


////////////////////////////////////////////////////////////////////////////////////////////////////


typedef std::vector<std::pair<double, CostFunction>> WeightedFunctions;

double WeightedCostFunction(const WeightedFunctions & weigtedFunctions,
                            const PolyFunction & sTraj, const PolyFunction & dTraj) {
  double result = 0;
  for (const auto & wf : weigtedFunctions) {
    result += wf.first * wf.second(sTraj, dTraj);
  }
  return result;
}

double AllGoodFunction(const PolyFunction & sTraj, const PolyFunction & dTraj) {
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
  
  if (maxSpeedSoFar < speedLimit * percent) {
    return 0.0;
  }

  double a = (1-percent) * speedLimit;
  double k = speedLimit / a;
  return k * maxSpeedSoFar - speedLimit;
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
  size_t totalIntervals = 100;
  double step = targetTime / totalIntervals;

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


int DPosToCurrentLane(double d, double laneWidth) {
  return d / laneWidth;
}

double CurrentLaneToDPos(int laneIdx, double laneWidth) {
  return laneWidth + laneWidth / 2;
}


Decider::Decider(double horizonSeconds, double laneWidth, double minTrajectoryTimeSeconds)
: m_horizonSeconds(horizonSeconds), m_laneWidth(laneWidth), m_minTrajectoryTimeSeconds(minTrajectoryTimeSeconds), m_state(0){
}

std::pair<PolyFunction, PolyFunction> Decider::ChooseBestTrajectory(const State2D & startState) {
  const int kKeepSpeedState = 0;
  const int kFollowVehicleState = 1;
  const int kChangingLaneLeftState = 2;
  const int kChangingLaneRightState = 3;

  const double kTimeStep = 0.5;
  
  const auto outsideOfTheRoadPenalty = [this](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    return OutsideOfTheRoadPenalty(sTraj, dTraj, 0 + m_laneWidth / 4, 3 * m_laneWidth - m_laneWidth / 4, targetTime);
  };
  
  // README: Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

  const auto accelerationLimit = [](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    const double kMaxAccelerationMs2 = 10;
    return AccelerationLimitCost(sTraj, dTraj, targetTime, kMaxAccelerationMs2);
  };
  
  const auto speedLimit = [](const PolyFunction & sTraj, const PolyFunction & dTraj, double targetTime) {
    return SpeedLimitCost(sTraj, dTraj, targetTime, MiphToMs(50));
  };

  World world;
  
  using std::placeholders::_1;
  using std::placeholders::_2;
  
  switch (m_state) {
    case kKeepSpeedState: {
      const double kTargetSpeed = MiphToMs(40);
      const double targetTime = m_horizonSeconds * 3;

      const int kTargetLaneIdx = DPosToCurrentLane(startState.d.s, m_laneWidth);
      const double kTargetLaneD = CurrentLaneToDPos(kTargetLaneIdx, m_laneWidth);

      ConstantSpeedTarget target(kTargetSpeed, startState.s.s, State{kTargetLaneD, 0, 0});
      
      WeightedFunctions weighted{
        {1, std::bind(ClosenessToTargetSState, _1, _2, target, targetTime)},
        {1, std::bind(ClosenessToTargetDState, _1, _2, target, targetTime)},
        {70, std::bind(speedLimit, _1, _2, targetTime)},
        {70, std::bind(accelerationLimit, _1, _2, targetTime)},
        {1000, std::bind(outsideOfTheRoadPenalty, _1, _2, targetTime)}
      };

      auto costFunction = std::bind(WeightedCostFunction, weighted, _1, _2);
      
      return FindBestTrajectories(startState, target, m_minTrajectoryTimeSeconds, targetTime, kTimeStep, costFunction);
    } break;
    case kFollowVehicleState: {
      
    } break;
    case kChangingLaneLeftState: {
      
    } break;
    case kChangingLaneRightState: {
      
    } break;
    default:
      break;
  }
  
  throw std::runtime_error("Must not reach");
}


////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {
  const double kHorizonSeconds = 5;
  const double kReplanPeriodSeconds = 1;
  const int kPointsToKeep = 0;
  
  // TODO: it is possible that a generated trajectory would end earlier than we replan, it is not supported now, this is why minTime
  // is set to be higher than replan frequency.
  const double kMinTrajectoryTimeSeconds = kReplanPeriodSeconds + 0.5;
} // namespace

Planner::Planner(const Map& map, double updatePeriodSeconds, double laneWidthMeters)
    : m_updatePeriod(updatePeriodSeconds),
      m_laneWidth(laneWidthMeters),
      m_map(map),
      m_decider(kHorizonSeconds, laneWidthMeters, kMinTrajectoryTimeSeconds),
      m_trajectoryOffsetIdx(0),
      m_hasTrajectory(false),
      m_updateNumber(0)
{}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCar>& sensors) {
  
  bool isTimeToReplan = m_updateNumber == 0;

  ssize_t currentPosIdx = m_plannedPath.size() - unprocessedPath.size();

  bool continueTrajectory = true;
  
  // Handle huge lag.
  if (unprocessedPath.empty()) {
    continueTrajectory = false;
    currentPosIdx = 0;
    m_trajectoryOffsetIdx = 0;
    isTimeToReplan = true;
  }

  if (currentPosIdx * m_updatePeriod > kReplanPeriodSeconds) {
    isTimeToReplan = true;
  }
  
  std::cout << "idx=" << currentPosIdx << ", s=" << car.fp.s << ", d=" << car.fp.d << ", speed=" << car.car.speed << (isTimeToReplan ? ", replanning" : ",") << std::endl;
  
  if (!isTimeToReplan) {
    return unprocessedPath;
  }

  // Replan trajectories
  
  State2D startState{State{car.fp.s, car.car.speed, 0}, State{car.fp.d, 0, 0}};

  if (m_hasTrajectory && continueTrajectory) {
    ssize_t nextPosIdx = currentPosIdx + kPointsToKeep - m_trajectoryOffsetIdx;

    startState.s.s = m_plannedTrajectoryS.Eval(nextPosIdx * m_updatePeriod);
    startState.s.v = m_plannedTrajectoryS.Eval2(nextPosIdx * m_updatePeriod);
    startState.s.acc = m_plannedTrajectoryS.Eval3(nextPosIdx * m_updatePeriod);
    
    startState.d.s = m_plannedTrajectoryD.Eval(nextPosIdx * m_updatePeriod);
    startState.d.v = m_plannedTrajectoryD.Eval2(nextPosIdx * m_updatePeriod);
    startState.d.acc = m_plannedTrajectoryD.Eval3(nextPosIdx * m_updatePeriod);
  }
  
  auto bestTrajectory = m_decider.ChooseBestTrajectory(startState);

//  std::vector<double> plannedS;

  std::vector<Point> planned;
  if (continueTrajectory && (currentPosIdx + kPointsToKeep - 1 < m_plannedPath.size())) {
    // append points from previous trajectory so that transition between trajectories would be smooth
    planned.insert(planned.begin(), m_plannedPath.begin() + currentPosIdx, m_plannedPath.begin() + currentPosIdx + kPointsToKeep);
    m_trajectoryOffsetIdx = planned.size();
  }

  for (int i = 0; i < kHorizonSeconds / m_updatePeriod; ++i) {
    FrenetPoint pt = car.fp;
    pt.s = bestTrajectory.first.Eval(i * m_updatePeriod);
    pt.d = bestTrajectory.second.Eval(i * m_updatePeriod);
//    plannedS.push_back(pt.s);
    planned.push_back(m_map.FromFrenet(pt));
  }

//  std::cout << "# " << m_updateNumber << std::endl;
//  for (int i = 1; i < planned.size(); ++i) {
//    double dist = Distance(planned[i-1].x, planned[i-1].y, planned[i].x, planned[i].y);
//    std::cout << plannedS[i] << "\t" << plannedS[i] - plannedS[i-1] << "\t" << dist/m_updatePeriod << "\n";
//  }
//  std::cout << std::endl;

  m_plannedPath = planned;
//  std::cout << "Sending " << m_plannedPath.size() << " points" << std::endl;

  m_plannedTrajectoryS = bestTrajectory.first;
  m_plannedTrajectoryD = bestTrajectory.second;
  m_hasTrajectory = true;
  m_updateNumber++;
  return m_plannedPath;
}
