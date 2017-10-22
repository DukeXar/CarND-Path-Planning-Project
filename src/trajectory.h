#pragma once

#include <array>
#include <functional>
#include "map.h"

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

PolyFunction JerkMinimizingTrajectory(const State& start, const State& end,
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

const double kSigmaSAcc = 2.0;
const double kSigmaSV = 4.0;
const double kSigmaSS = 10.0;

const double kSigmaDAcc = 1.0;
const double kSigmaDV = 1.0;
const double kSigmaDS = 1.0;

typedef std::function<double(const PolyFunction& sTraj,
                             const PolyFunction& dTraj, double targetTime)>
    CostFunction;

struct BestTrajectories {
  PolyFunction s, d;
  double time;
  double cost;
};

BestTrajectories FindBestTrajectories(const State2D& start,
                                      const Target& target,
                                      const GenConfig& config,
                                      const CostFunction& costFunction);
