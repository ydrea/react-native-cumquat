#include "Projection.h"

#include <algorithm>
#include <cmath>

namespace cumquat::projection {
namespace {
constexpr double kPi = 3.14159265358979323846;

inline double radians(double degrees) {
  return degrees * kPi / 180.0;
}

Vec3 rotateByQuaternion(const Vec3& vector, const Quaternion& source) {
  const double norm = std::sqrt(
      source.x * source.x + source.y * source.y +
      source.z * source.z + source.w * source.w);

  const double inverseNorm = norm > 0.0 ? 1.0 / norm : 1.0;
  const double qx = source.x * inverseNorm;
  const double qy = source.y * inverseNorm;
  const double qz = source.z * inverseNorm;
  const double qw = source.w * inverseNorm;

  const double tx = 2.0 * (qy * vector.z - qz * vector.y);
  const double ty = 2.0 * (qz * vector.x - qx * vector.z);
  const double tz = 2.0 * (qx * vector.y - qy * vector.x);

  return {
      vector.x + qw * tx + (qy * tz - qz * ty),
      vector.y + qw * ty + (qz * tx - qx * tz),
      vector.z + qw * tz + (qx * ty - qy * tx),
  };
}

Quaternion worldToCameraOrientation(
    const Quaternion& deviceOrientation) {
  return {
      -deviceOrientation.x,
      -deviceOrientation.y,
      -deviceOrientation.z,
      deviceOrientation.w,
  };
}
} // namespace

Vec3 worldToCamera(const Vec3& enu, const SensorState& sensorState) {
  if (sensorState.hasOrientationQuaternion) {
    return rotateByQuaternion(
        enu,
        worldToCameraOrientation(sensorState.orientation));
  }

  const double heading = radians(sensorState.headingDeg);
  const double pitch = radians(-sensorState.pitchDeg);
  const double roll = radians(-sensorState.rollDeg);

  const double ch = std::cos(heading);
  const double sh = std::sin(heading);
  const Vec3 yawed{
      ch * enu.x - sh * enu.y,
      sh * enu.x + ch * enu.y,
      enu.z,
  };

  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const Vec3 pitched{
      yawed.x,
      cp * yawed.y - sp * yawed.z,
      sp * yawed.y + cp * yawed.z,
  };

  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  return {
      cr * pitched.x - sr * pitched.z,
      pitched.y,
      sr * pitched.x + cr * pitched.z,
  };
}

bool projectToScreen(
    const Vec3& camera,
    const SensorState& sensorState,
    const ViewState& viewState,
    double& x,
    double& y,
    double& depth) {
  const double width = sensorState.viewportWidth;
  const double height = sensorState.viewportHeight;
  if (width <= 0.0 || height <= 0.0) return false;

  if (sensorState.hasOrientationQuaternion) {
    const double forwardDepth = -camera.z;
    depth = forwardDepth;

    if (forwardDepth <= 0.1) {
      x = width * 0.5;
      y = height * 0.5;
      return false;
    }

    const double focal =
        width / (2.0 * std::tan(radians(viewState.horizontalFovDeg) * 0.5));
    const double cameraRight = -camera.y;
    const double cameraUp = -camera.x;

    x = width * 0.5 + cameraRight * focal / forwardDepth;
    y = height * 0.5 - cameraUp * focal / forwardDepth;

    return x >= 0.0 && x <= width && y >= 0.0 && y <= height;
  }

  const double focal =
      width / (2.0 * std::tan(radians(viewState.horizontalFovDeg) * 0.5));
  depth = camera.y;

  if (depth <= 0.1) {
    const double directionDepth = std::max(camera.length(), 0.1);
    x = width * 0.5 + camera.x * focal / directionDepth;
    y = height * 0.5 - camera.z * focal / directionDepth;
    return false;
  }

  x = width * 0.5 + camera.x * focal / depth;
  y = height * 0.5 - camera.z * focal / depth;
  return x >= 0.0 && x <= width && y >= 0.0 && y <= height;
}

} // namespace cumquat::projection
