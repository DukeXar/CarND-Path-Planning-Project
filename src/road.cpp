#include <fstream>
#include <iostream>
#include "map.h"
#include "planner.h"

std::ostream& operator<<(std::ostream& os, const Decider::Mode& mode) {
  switch (mode) {
    case Decider::Mode::kChangingLane:
      os << "changing lanes";
      break;
    case Decider::Mode::kFollowVehicle:
      os << "following";
      break;
    case Decider::Mode::kKeepSpeed:
      os << "keeping speed";
      break;
    default:
      os << "unknown";
      break;
  }
  return os;
}

int main() {
  const std::string MAP_FILENAME = "../data/highway_map.csv";
  Map map;
  ReadMap(MAP_FILENAME, map);

  std::cout << "Loaded map " << MAP_FILENAME << ", total " << map.get_size()
            << " points" << std::endl;

  const double laneWidth = 4;
  const double minTrajectoryTime = 2;
  const double latency = 1.5;
  const double step = 0.02;

  State2D prevPoint = State2D{State{376.11, 0, 0}, State{6.00, 0.00, 0.01}};
  // State2D prevPoint = State2D{State{176.11, 0, 0}, State{6.00, 0.00, 0.01}};
  Decider decider(laneWidth, minTrajectoryTime, latency, map);

  std::ofstream ofile("s.txt");
  ofile << "pos;time;s_pos;s_speed;s_acc;d_pos;cost;mode" << std::endl;

  double otherCarSpeed = 15;
  OtherCarSensor otherCar;
  otherCar.id = 1;
  otherCar.pos.x = 0;
  otherCar.pos.y = 0;
  otherCar.speed.x = otherCarSpeed;
  otherCar.speed.y = 1;
  otherCar.fnPos.s = 376.11;
  otherCar.fnPos.d = 6.00;

  double currTime = 0;
  double prevTime = 0;
  // for (int pos = 0; pos < 10; ++pos) {
  //   otherCar.fnPos.s += otherCarSpeed * (prevTime - currTime);
  //   BestTrajectories best = decider.ChooseBestTrajectory(
  //       prevPoint, std::vector<OtherCarSensor>{otherCar});

  //   double t = 0;
  //   prevTime = currTime;
  //   while (t <= best.time) {
  //     State2D currPoint{
  //         State{best.s.Eval(t), best.s.Eval2(t), best.s.Eval3(t)},
  //         State{best.d.Eval(t), best.d.Eval2(t), best.d.Eval3(t)}};

  //     ofile << pos << ";" << currTime + t << ";" << currPoint.s.s << ";"
  //           << currPoint.s.v << ";" << currPoint.s.acc << ";" <<
  //           currPoint.d.s
  //           << std::endl;

  //     t += step;

  //     currTime += step;
  //     prevPoint = currPoint;
  //   }
  // }

  for (int pos = 0; pos < 50; ++pos) {
    otherCar.fnPos.s += otherCarSpeed * (currTime - prevTime);
    BestTrajectories best = decider.ChooseBestTrajectory(
        prevPoint, std::vector<OtherCarSensor>{otherCar});

    double t = 0;
    prevTime = currTime;
    while (t <= 0.5) {
      State2D currPoint{
          State{best.s.Eval(t), best.s.Eval2(t), best.s.Eval3(t)},
          State{best.d.Eval(t), best.d.Eval2(t), best.d.Eval3(t)}};

      ofile << pos << ";" << currTime + t << ";" << currPoint.s.s << ";"
            << currPoint.s.v << ";" << currPoint.s.acc << ";" << currPoint.d.s
            << best.cost << ";" << decider.GetMode() << std::endl;

      t += step;

      currTime += step;
      prevPoint = currPoint;
    }
  }

  ofile.close();

  return 0;
}