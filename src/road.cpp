#include <fstream>
#include <iostream>
#include "map.h"
#include "planner.h"

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

  State2D prevPoint =
      State2D{State{176.11, 20.86, 3.82}, State{6.00, 0.00, 0.01}};
  Decider decider(laneWidth, minTrajectoryTime, latency, map);

  std::ofstream ofile("s.txt");
  ofile << "pos;time;s_pos;s_speed;s_acc;d_pos" << std::endl;

  double currTime = 0;
  for (int pos = 0; pos < 10; ++pos) {
    BestTrajectories best =
        decider.ChooseBestTrajectory(prevPoint, std::vector<OtherCarSensor>{});

    double t = 0;
    while (t <= best.time) {
      State2D currPoint{
          State{best.s.Eval(t), best.s.Eval2(t), best.s.Eval3(t)},
          State{best.d.Eval(t), best.d.Eval2(t), best.d.Eval3(t)}};

      ofile << pos << ";" << currTime + t << ";" << currPoint.s.s << ";"
            << currPoint.s.v << ";" << currPoint.s.acc << ";" << currPoint.d.s
            << std::endl;

      t += step;

      currTime += step;
      prevPoint = currPoint;
    }
  }

  ofile.close();

  return 0;
}