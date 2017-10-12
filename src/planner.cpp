#include "planner.h"
#include <iostream>
#include <array>
#include <stdexcept>
#include <functional>
#include <random>
#include <algorithm>
#include "Dense"
#include "map.h"
#include "utils.h"

void JerkMinimizingTrajectory::Fit() {
  const double t = m_time;
  const double t2 = t * t;
  const double t3 = t * t * t;
  const double t4 = t2 * t2;

  Eigen::MatrixXd a = Eigen::MatrixXd(3, 3);
  a <<
    t3, t4, t2 * t3,
    3 * t2, 4 * t3, 5 * t4,
    6 * t, 12 * t2, 20 * t3;
  
  const double ds = m_end.s - (m_start.s + m_start.v * t + .5 * m_start.acc * t2);
  const double dv = m_end.v - (m_start.v + m_start.acc * t);
  const double dacc = m_end.acc - m_start.acc;

  Eigen::MatrixXd b = Eigen::MatrixXd(3, 1);
  b << ds, dv, dacc;
  
  Eigen::MatrixXd c = a.inverse() * b;
  m_coeff = {m_start.s, m_start.v, 0.5 * m_start.acc, c.data()[0], c.data()[1], c.data()[2]};
  m_fit = true;
}

void JerkMinimizingTrajectory::AssertFit() const {
  if (!m_fit) {
    throw std::runtime_error("JerkMinimizingTrajectory::Fit() must be called before Eval(time)");
  }
}

double JerkMinimizingTrajectory::Eval(double time) const {
  AssertFit();

  double k = 1;
  double res = 0;
  for (const auto & c : m_coeff) {
    res += k * c;
    k *= time;
  }

  return res;
}

double JerkMinimizingTrajectory::Eval2(double time) const {
  // s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  // s'(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t^2 + 4 * a_4 * t^3 + 5 * a_5 * t^4
  AssertFit();
  
  return m_coeff[1] + 2 * m_coeff[2] * time + 3 * m_coeff[3] * time * time + 4 * m_coeff[4] * time * time * time + 5 * m_coeff[5] * time * time * time * time;
}

double JerkMinimizingTrajectory::Eval3(double time) const {
  AssertFit();
  // s'(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t^2 + 4 * a_4 * t^3 + 5 * a_5 * t^4
  // s''(t) =      2 * a_2     + 6 * a_3 * t  + 12 * a_4 * t^2 + 20 * a_5 * t^3
  
  return 2 * m_coeff[2] + 6 * m_coeff[3] * time + 12 * m_coeff[4] * time * time + 20 * m_coeff[5] * time * time * time;
}


////////////////////////////////////////////////////////////////////////////////////////////////////


struct State2D {
  State s;
  State d;
};

struct Goal2D {
  State2D state;
  double time;
};


////////////////////////////////////////////////////////////////////////////////////////////////////


const double kSigmaSAcc = 2.0;
const double kSigmaSV = 4.0;
const double kSigmaSS = 10.0;

const double kSigmaDAcc = 1.0;
const double kSigmaDV = 1.0;
const double kSigmaDS = 1.0;


State2D PerturbTarget(const State2D & target) {
  std::random_device rd;
  std::mt19937 gen(rd());
  
  State resS;
  {
    std::normal_distribution<> d(target.s.s, kSigmaSS);
    resS.s = d(gen);
  }
  {
    std::normal_distribution<> d(target.s.v, kSigmaSV);
    resS.v = d(gen);
  }
  {
    std::normal_distribution<> d(target.s.acc, kSigmaSAcc);
    resS.acc = d(gen);
  }
  
  State resD;
  {
    std::normal_distribution<> d(target.d.s, kSigmaDS);
    resD.s = d(gen);
  }
  {
    std::normal_distribution<> d(target.d.v, kSigmaDV);
    resD.v = d(gen);
  }
  {
    std::normal_distribution<> d(target.d.acc, kSigmaDAcc);
    resD.acc = d(gen);
  }
  return {resS, resD};
}


////////////////////////////////////////////////////////////////////////////////////////////////////


class Target {
public:
  virtual State2D At(double time) const = 0;
};

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

typedef std::function<double(const JerkMinimizingTrajectory & sTraj,
                     const JerkMinimizingTrajectory & dTraj)> CostFunction;


double WeightedCostFunction(const std::vector<std::pair<double, CostFunction>> & weigtedFunctions,
                            const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj) {
  double result = 0;
  for (const auto & wf : weigtedFunctions) {
    result += wf.first * wf.second(sTraj, dTraj);
  }
  return result;
}

double AllGoodFunction(const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj) {
  return 1.0;
}

double SpeedLimitFunction(const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj,
                          double targetTime, double speedLimit) {
  double maxSpeedSoFar = 0;
  size_t totalIntervals = 100;
  double intervalLength = targetTime / totalIntervals;
  for (int i = 1; i < totalIntervals; ++i) {
    double speed = sTraj.Eval2(i * intervalLength);
    maxSpeedSoFar = std::max(maxSpeedSoFar, speed);
  }

  if (maxSpeedSoFar > speedLimit) {
    return 0.0;
  }

  return (speedLimit - maxSpeedSoFar) / speedLimit;
}

double Logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double ClosenessCost(double x1, double x2, double sigma) {
  return Logistic(std::abs(x1 - x2) / sigma);
}

double ClosenessToTargetSState(const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj,
                               const Target & target, double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;

  cost += ClosenessCost(targetState.s.s, sTraj.Eval(targetTime), kSigmaSS);
  cost += ClosenessCost(targetState.s.v, sTraj.Eval2(targetTime), kSigmaSV);
  cost += ClosenessCost(targetState.s.acc, sTraj.Eval3(targetTime), kSigmaSAcc);
  
  return cost;
}

double ClosenessToTargetDState(const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj,
                               const Target & target, double targetTime) {
  auto targetState = target.At(targetTime);

  double cost = 0;
  
  cost += ClosenessCost(targetState.d.s, dTraj.Eval(targetTime), kSigmaDS);
  cost += ClosenessCost(targetState.d.v, dTraj.Eval2(targetTime), kSigmaDV);
  cost += ClosenessCost(targetState.d.acc, dTraj.Eval3(targetTime), kSigmaDAcc);
  
  return cost;
}


/////////////////////////////////////////////////////////////////////////////////////////


typedef std::unique_ptr<JerkMinimizingTrajectory> TrajectoryPtr;

std::pair<TrajectoryPtr, TrajectoryPtr> FindBestTrajectory(const State2D & start,
                                                           const Target & target,
                                                           double targetTime,
                                                           double timeDelta,
                                                           const CostFunction & costFunction) {
  
  const int kSamples = 10;
  
  std::vector<Goal2D> goals;
  
  double currTime = timeDelta;
  while (currTime <= targetTime + 4 * timeDelta) {
    State2D currTarget = target.At(currTime);
    goals.push_back({currTarget, currTime});
    
    for (int sample = 0; sample < kSamples; ++sample) {
      State2D perturbedTarget = PerturbTarget(currTarget);
      goals.push_back({perturbedTarget, currTime});
    }

    currTime += timeDelta;
  }

  std::vector<std::pair<TrajectoryPtr, TrajectoryPtr>> trajectories;
  
  for (const auto & goal : goals) {
    TrajectoryPtr sTraj(new JerkMinimizingTrajectory(start.s, goal.state.s, goal.time));
    sTraj->Fit();
    TrajectoryPtr dTraj(new JerkMinimizingTrajectory(start.d, goal.state.d, goal.time));
    dTraj->Fit();
    trajectories.emplace_back(std::move(sTraj), std::move(dTraj));
  }
  
  std::vector<double> allCosts;
  for (const auto & sdTraj : trajectories) {
    allCosts.push_back(costFunction(*sdTraj.first, *sdTraj.second));
  }
  
//  std::cout << "allCosts=[";
//  for (auto v : allCosts) std::cout << " " << v;
//  std::cout << "]" << std::endl;

  auto bestIdx = std::min_element(begin(allCosts), end(allCosts)) - begin(allCosts);
  
  return std::move(trajectories[bestIdx]);
}


////////////////////////////////////////////////////////////////////////////////////////////////////


Planner::Planner(const Map& map, double updatePeriodSeconds,
                 double laneWidthMeters)
    : m_updatePeriod(updatePeriodSeconds),
      m_laneWidth(laneWidthMeters),
      m_map(map),
      m_updateNumber(0) {}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCar>& sensors) {
  const double kTargetSpeed = MiphToMs(30);

  const double kHorizonSeconds = 5;
  const double kReplanPeriodSeconds = 1;

  const int kHorizonCount = kHorizonSeconds / m_updatePeriod;

  const int currentLane = 2;
  const double currentLaneD = currentLane * m_laneWidth + m_laneWidth / 2;

  double targetTime = 10;

  bool timeToReplan = m_updateNumber == 0;

  std::cout << "Received " << unprocessedPath.size() << " points" << std::endl;
  ssize_t currentPosIdx = m_plannedPath.size() - unprocessedPath.size();
  
  bool continueTrajectory = true;
  
  // WTF?
  if (unprocessedPath.empty()) {
    continueTrajectory = false;
    currentPosIdx = 0;
    timeToReplan = true;
  }

  if (currentPosIdx * m_updatePeriod > kReplanPeriodSeconds) {
    timeToReplan = true;
  }
  
  std::cout << "processed=" << currentPosIdx << ", currentS=" << car.fp.s << ", speed=" << car.car.speed << (timeToReplan ? ", replanning" : ", --") << std::endl;
  
  if (!timeToReplan) {
    return unprocessedPath;
  }

  State2D startState{State{car.fp.s, car.car.speed, 0}, State{car.fp.d, 0, 0}};

  if (m_plannedTrajectoryS && continueTrajectory) {
    ssize_t nextPosIdx = currentPosIdx + 2;

    startState.s.s = m_plannedTrajectoryS->Eval(nextPosIdx * m_updatePeriod);
    startState.s.v = m_plannedTrajectoryS->Eval2(nextPosIdx * m_updatePeriod);
    startState.s.acc = m_plannedTrajectoryS->Eval3(nextPosIdx * m_updatePeriod);
    
    startState.d.s = m_plannedTrajectoryD->Eval(nextPosIdx * m_updatePeriod);
    startState.d.v = m_plannedTrajectoryD->Eval2(nextPosIdx * m_updatePeriod);
    startState.d.acc = m_plannedTrajectoryD->Eval3(nextPosIdx * m_updatePeriod);
  }
  
  ConstantSpeedTarget target(kTargetSpeed, startState.s.s, State{currentLaneD, 0, 0});

  World world;
  
  auto costFunction = [targetTime, target, &world](const JerkMinimizingTrajectory & sTraj, const JerkMinimizingTrajectory & dTraj) {
    return ClosenessToTargetSState(sTraj, dTraj, target, targetTime) + ClosenessToTargetDState(sTraj, dTraj, target, targetTime);
  };
  
  auto bestTrajectory = FindBestTrajectory(startState, target, targetTime, 0.5, costFunction);

  std::vector<double> plannedS;

  std::vector<Point> planned;
  if (continueTrajectory && (currentPosIdx + 1 < m_plannedPath.size())) {
    // not the first time
    planned.push_back(m_plannedPath[currentPosIdx]);
    planned.push_back(m_plannedPath[currentPosIdx+1]);
  }
  
  for (int i = 0; i < kHorizonCount; ++i) {
    FrenetPoint pt = car.fp;
    pt.s = bestTrajectory.first->Eval(i * m_updatePeriod);
    pt.d = bestTrajectory.second->Eval(i * m_updatePeriod);
    plannedS.push_back(pt.s);
    planned.push_back(m_map.FromFrenet(pt));
  }

//  std::cout << "# " << m_updateNumber << std::endl;
//  for (int i = 1; i < planned.size(); ++i) {
//    double dist = Distance(planned[i-1].x, planned[i-1].y, planned[i].x, planned[i].y);
//    std::cout << plannedS[i] << "\t" << plannedS[i] - plannedS[i-1] << "\t" << dist/m_updatePeriod << "\n";
//  }
//  std::cout << std::endl;

  m_plannedPath = planned;
  std::cout << "Sending " << m_plannedPath.size() << " points" << std::endl;

  m_plannedTrajectoryS.swap(bestTrajectory.first);
  m_plannedTrajectoryD.swap(bestTrajectory.second);
  m_updateNumber++;
  return m_plannedPath;
}
