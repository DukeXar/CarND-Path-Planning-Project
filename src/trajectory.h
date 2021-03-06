#pragma once

#include <array>
#include <functional>
#include <utility>
#include <vector>
#include "map.h"

struct State {
  double s;
  double v;
  double acc;
};

class PolyFunction {
 public:
  PolyFunction() {}
  explicit PolyFunction(const std::array<double, 6>& coeff) : m_coeff(coeff) {}

  double Eval(double x) const;
  double Eval2(double x) const;
  double Eval3(double x) const;

 private:
  std::array<double, 6> m_coeff;
};

PolyFunction JerkMinimizingTrajectory(const State& start,
                                      const State& end,
                                      double time);

PolyFunction JerkMinimizingTrajectory4(const State& start,
                                       const State& end,
                                       double time);

struct State2D {
  State s;
  State d;
};

class Target {
 public:
  virtual State2D At(double time) const = 0;
};

struct GenConfig {
  State sigmaS;
  State sigmaD;
  int samplesCount;
  double minTime;
  double maxTime;
  double timeStep;
};

struct Goal2D {
  State2D state;
  double time;
};

const double kSigmaSAcc = 2.0;
const double kSigmaSV = 4.0;
const double kSigmaSS = 10.0;

const double kSigmaDAcc = 1.0;
const double kSigmaDV = 1.0;
const double kSigmaDS = 0.5;

typedef std::function<double(const PolyFunction& sTraj,
                             const PolyFunction& dTraj,
                             double targetTime)>
    CostFunction;

typedef std::vector<std::pair<double, CostFunction>> WeightedFunctions;

struct BestTrajectories {
  PolyFunction s, d;
  double time;
  double cost;
  std::vector<double> detailedCost;
};

State2D PerturbTarget(const State2D& target,
                      const State& sigmaS,
                      const State& sigmaD);

BestTrajectories FindBestTrajectories(const State2D& start,
                                      const Target& target,
                                      const GenConfig& config,
                                      const WeightedFunctions& costFunctions,
                                      bool noTargetLongitudialPos = false);

BestTrajectories FindBestTrajectories(const State2D& start,
                                      const std::vector<Goal2D>& goals,
                                      const WeightedFunctions& costFunctions,
                                      bool noTargetLongitudialPos = false);