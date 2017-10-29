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
  Decider decider(laneWidth, minTrajectoryTime, latency, map);
  BestTrajectories best = decider.ChooseBestTrajectory(
      State2D{State{552.96, 17.65, 4.88}, State{5.36, -0.11, 2.15}},
      std::vector<OtherCarSensor>{});

  std::ofstream ofile("s.txt");

  double t = 0;
  ofile << "time;s_pos;s_speed;s_acc;d_pos" << std::endl;
  while (t <= best.time) {
    ofile << t << ";" << best.s.Eval(t) << ";" << best.s.Eval2(t) << ";"
          << best.s.Eval3(t) << ";" << best.d.Eval(t) << std::endl;
    t += step;
  }
  ofile.close();

  return 0;
}