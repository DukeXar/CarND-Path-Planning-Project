#include "trajectory.h"

#include <iostream>
#include <random>
#include "Dense"


PolyFunction JerkMinimizingTrajectory(const State & start, const State & end, double time) {
  const double t = time;
  const double t2 = t * t;
  const double t3 = t * t * t;
  const double t4 = t2 * t2;
  
  Eigen::MatrixXd a = Eigen::MatrixXd(3, 3);
  a <<
  t3, t4, t2 * t3,
  3 * t2, 4 * t3, 5 * t4,
  6 * t, 12 * t2, 20 * t3;
  
  const double ds = end.s - (start.s + start.v * t + .5 * start.acc * t2);
  const double dv = end.v - (start.v + start.acc * t);
  const double dacc = end.acc - start.acc;
  
  Eigen::MatrixXd b = Eigen::MatrixXd(3, 1);
  b << ds, dv, dacc;
  
  Eigen::MatrixXd c = a.inverse() * b;
  return PolyFunction({start.s, start.v, 0.5 * start.acc, c.data()[0], c.data()[1], c.data()[2]});
}

double PolyFunction::Eval(double x) const {
  double k = 1;
  double res = 0;
  for (const auto & c : m_coeff) {
    res += k * c;
    k *= x;
  }
  
  return res;
}

double PolyFunction::Eval2(double x) const {
  // s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  // s'(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t^2 + 4 * a_4 * t^3 + 5 * a_5 * t^4
  
  return m_coeff[1] + 2 * m_coeff[2] * x + 3 * m_coeff[3] * x * x + 4 * m_coeff[4] * x * x * x + 5 * m_coeff[5] * x * x * x * x;
}

double PolyFunction::Eval3(double x) const {
  // s'(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t^2 + 4 * a_4 * t^3 + 5 * a_5 * t^4
  // s''(t) =      2 * a_2     + 6 * a_3 * t  + 12 * a_4 * t^2 + 20 * a_5 * t^3
  
  return 2 * m_coeff[2] + 6 * m_coeff[3] * x + 12 * m_coeff[4] * x * x + 20 * m_coeff[5] * x * x * x;
}

struct Goal2D {
  State2D state;
  double time;
};

State2D PerturbTarget(const State2D & target, const State & sigmaS, const State & sigmaD) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  
  State resS;
  {
    std::normal_distribution<> d(target.s.s, sigmaS.s);
    resS.s = d(gen);
  }
  {
    std::normal_distribution<> d(target.s.v, sigmaS.v);
    resS.v = d(gen);
  }
  {
    std::normal_distribution<> d(target.s.acc, sigmaS.acc);
    resS.acc = d(gen);
  }
  
  State resD;
  {
    std::normal_distribution<> d(target.d.s, sigmaD.s);
    resD.s = d(gen);
  }
  {
    std::normal_distribution<> d(target.d.v, sigmaD.v);
    resD.v = d(gen);
  }
  {
    std::normal_distribution<> d(target.d.acc, sigmaD.acc);
    resD.acc = d(gen);
  }
  return {resS, resD};
}

BestTrajectories FindBestTrajectories(const State2D & start,
                                      const Target & target,
                                      const GenConfig & config,
                                      const CostFunction & costFunction) {
  
  std::vector<Goal2D> goals;
  
  double currTime = config.minTime;
  while (currTime <= config.maxTime) {
    State2D currTarget = target.At(currTime);
    goals.push_back({currTarget, currTime});

    int totalGenerated = 0;
    while (totalGenerated < config.samplesCount) {
      State2D perturbedTarget = PerturbTarget(currTarget, config.sigmaS, config.sigmaD);
      // Ensure we don't move backwards, and speed is sane.
      if (perturbedTarget.s.s > start.s.s && perturbedTarget.s.v >= 0) {
        goals.push_back({perturbedTarget, currTime});
        ++totalGenerated;
      }
    }
    
    currTime += config.timeStep;
  }
  
  std::vector<BestTrajectories> trajectories;
  
  for (const auto & goal : goals) {
    trajectories.push_back({
      JerkMinimizingTrajectory(start.s, goal.state.s, goal.time),
      JerkMinimizingTrajectory(start.d, goal.state.d, goal.time),
      goal.time
    });
  }
  
  std::vector<double> allCosts;
  for (const auto & traj : trajectories) {
    allCosts.push_back(costFunction(traj.s, traj.d, traj.time));
  }
  
  //  std::cout << "allCosts=[";
  //  for (auto v : allCosts) std::cout << " " << v;
  //  std::cout << "]" << std::endl;
  
  auto bestIdx = std::min_element(begin(allCosts), end(allCosts)) - begin(allCosts);
  
  std::cout << "best trajectory=[";
  std::cout << "s=[" << "s=" << goals[bestIdx].state.s.s << ", v=" << goals[bestIdx].state.s.v << ", acc=" << goals[bestIdx].state.s.acc << "], ";
  std::cout << "d=[" << "s=" << goals[bestIdx].state.d.s << ", v=" << goals[bestIdx].state.d.v << ", acc=" << goals[bestIdx].state.d.acc << "], ";
  std::cout << "time=" << goals[bestIdx].time << ", cost=" << allCosts[bestIdx] << std::endl;
  
  return std::move(trajectories[bestIdx]);
}
