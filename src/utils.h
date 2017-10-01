#pragma once

#include <cmath>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

const int SecondsInHour = 60 * 60;
const double MetersInMile = 1609.34;

inline double KmphToMs(double kmph) { return kmph * 1000.0 / SecondsInHour; }

inline double MiphToMs(double miph) {
  return miph * MetersInMile / SecondsInHour;
}