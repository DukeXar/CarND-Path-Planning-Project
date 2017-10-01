#include "planner.h"
#include <iostream>
#include "map.h"
#include "utils.h"

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
  const double kTargetSpeed = MiphToMs(10);

  const double kHorizonSeconds = 2;

  const int kHorizonCount = kHorizonSeconds / m_updatePeriod;

  const int currentLane = 0;
  const double currentLaneD = currentLane * m_laneWidth + m_laneWidth / 2;

  const double currentSpeed = car.car.speed;

  std::vector<Point> planned;

  std::cout << "endPath=" << endPath.s << ", " << endPath.d << std::endl;
  std::cout << "unprocessedPath=" << unprocessedPath.size() << std::endl;

  int horizon = 0;
  if (m_updateNumber < 10) {
    horizon = kHorizonCount;
  } else {
    horizon = kHorizonCount;
  }

  double acc = 0;
  if (currentSpeed < kTargetSpeed) {
    acc = (kTargetSpeed - currentSpeed) / kHorizonSeconds;
  } else {
    acc = (kTargetSpeed - currentSpeed) / kHorizonSeconds;
  }

  for (int i = 0; i < horizon; ++i) {
    FrenetPoint pt = car.fp;
    double t = i * m_updatePeriod;
    double v = currentSpeed + acc * t;
//    pt.s += currentSpeed * t + acc * t * t / 2;
    pt.s += kTargetSpeed * t;
    pt.d = currentLaneD;
//    std::cout << "s=" << pt.s << ", d=" << pt.d << ", v=" << v << " ";
    planned.push_back(m_map.FromFrenet(pt));
  }
  // std::cout << std::endl;

//  std::cout << "acc=" << acc << ", currentSpeed=" << currentSpeed << std::endl;

//  for (int i = 1; i < planned.size(); ++i) {
//    double dist = Distance(planned[i].x, planned[i].y, planned[i - 1].x,
//                           planned[i - 1].y);
//    double speed = dist / m_updatePeriod;
//    std::cout << speed << " ";
//  }
//  std::cout << std::endl;

  m_updateNumber++;
  return planned;
}
