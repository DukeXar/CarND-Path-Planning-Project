#include "world.h"
#include <stdexcept>

WorldSnapshot::WorldSnapshot(const std::vector<OtherCar>& sensors,
                             double laneWidth)
    : m_laneWidth(laneWidth) {
  m_byId.reserve(15);

  for (const auto& car : sensors) {
    int lane = DPosToCurrentLane(car.fnPos.d, laneWidth);
    if (lane >= 0) {
      m_cars[lane].push_back(car.id);
      if (car.id >= m_byId.size()) {
        m_byId.resize(car.id + 1);
      }
      m_byId[car.id] = car;
    }
  }
}

bool WorldSnapshot::GetClosestCar(int laneIdx,
                                  double s,
                                  OtherCar* result) const {
  if (laneIdx >= m_cars.size()) {
    return false;
  }

  std::vector<int> carsInLane = m_cars[laneIdx];
  std::sort(carsInLane.begin(), carsInLane.end(), [this](int id1, int id2) {
    return m_byId[id1].fnPos.s < m_byId[id2].fnPos.s;
  });

  const OtherCar example{0, 0, {s, 0}};
  auto lb = std::lower_bound(carsInLane.begin(), carsInLane.end(), example,
                             [this](int id1, const OtherCar& c2) {
                               const OtherCar& c1 = m_byId[id1];
                               return c1.fnPos.s < c2.fnPos.s;
                             });

  if (lb == carsInLane.end()) {
    return false;
  }

  if (result) {
    *result = m_byId[*lb];
  }

  return true;
}

OtherCar WorldSnapshot::GetCarById(int id) const {
  if (id >= m_byId.size()) {
    throw std::runtime_error("Not found ");
  }
  return m_byId[id];
}

World::World(const std::vector<OtherCarSensor>& sensors, double laneWidth)
    : m_laneWidth(laneWidth) {
  for (const auto& sensor : sensors) {
    double speed = std::sqrt(sensor.speed.x * sensor.speed.x +
                             sensor.speed.y * sensor.speed.y);
    const State dstate{sensor.fnPos.d, 0, 0};
    m_models[sensor.id] = std::unique_ptr<Target>(
        new ConstantSpeedTarget(speed, sensor.fnPos.s, dstate, 0, 0));
  }
}

const WorldSnapshot& World::Simulate(double time) {
  auto it = m_snapshots.find(time);
  if (it != m_snapshots.end()) {
    return *(it->second);
  }

  std::vector<OtherCar> cars;
  cars.reserve(m_models.size());
  for (auto& idAndModel : m_models) {
    auto state = idAndModel.second->At(time);
    OtherCar car{idAndModel.first, state.s.v,
                 FrenetPoint{state.s.s, state.d.s}};
    cars.push_back(car);
  }

  auto snapPtr =
      std::unique_ptr<WorldSnapshot>(new WorldSnapshot(cars, m_laneWidth));
  auto pos = m_snapshots.emplace(time, std::move(snapPtr)).first;
  return *(pos->second);
}

State2D ConstantSpeedTarget::At(double time) const {
  if (time < 0) {
    throw std::runtime_error("WHOA!");
  }
  double s = m_speed * (time + m_latency) + m_startS - m_distance;
  return State2D{State{s, m_speed, 0}, m_stateD};
}

State2D ConstantAccelerationTargetSpeedTarget::At(double time) const {
  if (time < 0) {
    throw std::runtime_error("WHOA!");
  }

  time += m_latency;

  // s = s0 + v0*t + a*t^2/2
  // v = v0 + a*t

  double speedDelta = m_targetSpeed - m_startV;
  double acc = speedDelta > 0 ? m_acc : -m_acc;
  double timeToSpeed = std::max(0.0, speedDelta / acc);
  double timeOnSpeed = std::max(0.0, time - timeToSpeed);

  double s = m_startS + m_startV * timeToSpeed +
             0.5 * acc * std::pow(timeToSpeed, 2) + timeOnSpeed * m_targetSpeed;
  double v = m_startV + acc * timeToSpeed;

  return State2D{State{s - m_distance, v, 0}, m_startD};
}
