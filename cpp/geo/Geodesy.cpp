#include "Geodesy.h"

#include <cmath>

namespace cumquat::geo {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

double radians(double degrees) {
  return degrees * kPi / 180.0;
}

double degrees(double radiansValue) {
  return radiansValue * 180.0 / kPi;
}
} // namespace

Vec3 toECEF(const GeoPoint& point) {
  const double lat = radians(point.latitudeDeg);
  const double lon = radians(point.longitudeDeg);
  const double sinLat = std::sin(lat);
  const double cosLat = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sinLat * sinLat);

  return {
      (n + point.altitudeMeters) * cosLat * std::cos(lon),
      (n + point.altitudeMeters) * cosLat * std::sin(lon),
      (n * (1.0 - kE2) + point.altitudeMeters) * sinLat,
  };
}

Vec3 ecefToENU(const Vec3& ecef, const GeoPoint& origin) {
  const Vec3 originECEF = toECEF(origin);
  const Vec3 delta = ecef - originECEF;
  const double lat = radians(origin.latitudeDeg);
  const double lon = radians(origin.longitudeDeg);
  const double sinLat = std::sin(lat);
  const double cosLat = std::cos(lat);
  const double sinLon = std::sin(lon);
  const double cosLon = std::cos(lon);

  return {
      -sinLon * delta.x + cosLon * delta.y,
      -sinLat * cosLon * delta.x - sinLat * sinLon * delta.y + cosLat * delta.z,
      cosLat * cosLon * delta.x + cosLat * sinLon * delta.y + sinLat * delta.z,
  };
}

double initialBearingDeg(const GeoPoint& from, const GeoPoint& to) {
  const double lat1 = radians(from.latitudeDeg);
  const double lat2 = radians(to.latitudeDeg);
  const double dLon = radians(to.longitudeDeg - from.longitudeDeg);
  const double y = std::sin(dLon) * std::cos(lat2);
  const double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
  double bearing = std::fmod(degrees(std::atan2(y, x)) + 360.0, 360.0);
  return bearing < 0.0 ? bearing + 360.0 : bearing;
}

} // namespace cumquat::geo
