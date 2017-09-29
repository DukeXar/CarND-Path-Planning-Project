#include "planner.h"
#include "map.h"

Planner::Planner(const Map& map, double updatePeriod)
    : m_updatePeriod(updatePeriod), m_map(map) {}

std::vector<Point> Planner::Update(const CarEx& car,
                                   const std::vector<Point>& unprocessedPath,
                                   const FrenetPoint& endPath,
                                   const std::vector<OtherCar>& sensors) {
  return m_plannedPath;
}