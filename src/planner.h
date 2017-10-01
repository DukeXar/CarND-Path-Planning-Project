#pragma once

#include "map.h"

struct CarEx {
  Car car;
  FrenetPoint fp;
};

struct OtherCar {
  int id;
  Point pos;
  Point speed;
  FrenetPoint fnPos;
};

class Planner {
 public:
  explicit Planner(const Map& map, double updatePeriodS,
                   double laneWidthMeters);

  std::vector<Point> Update(const CarEx& car,
                            const std::vector<Point>& unprocessedPath,
                            const FrenetPoint& endPath,
                            const std::vector<OtherCar>& sensors);

 private:
  const double m_updatePeriod;
  const double m_laneWidth;
  Map m_map;
  std::vector<Point> m_plannedPath;
  unsigned m_updateNumber;
};