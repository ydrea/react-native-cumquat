#include "Projection.h"

#include <algorithm>
#include <cmath>

namespace cumquat::projection {
namespace {
constexpr double kPi = 3.14159265358979323846;

inline double radians(double degrees) {
  return degrees * kPi / 180.0;
}

Quaternion normalized(const Quaternion& source) {
  const double norm = std::sqrt(
      source.x * source.x + source.y * source.y +
      source.z * source.z + source.w * source.w);
  const double scale = norm > 0.0 ? 1.0 / norm : 1.0;
  return {
      source.x * scale,
      source.y * scale,
      source.z * scale,
      source.w * scale,
  };
}

Quaternion multiply(const Quaternion& left, const Quaternion& right) {
  return {
      left.w * right.x + left.x * right.w +
          left.y * right.z - left.z * right.y,
      left.w * right.y - left.x * right.z +
          left.y * right.w + left.z * right.x,
      left.w * right.z + left.x * right.y -
          left.y * right.x + left.z * right.w,
      left.w * right.w - left.x * right.x -
          left.y * right.y - left.z * right.z,
  };
}

Quaternion inverse(const Quaternion& source) {
  const Quaternion q = normalized(source);
  return {-q.x, -q.y, -q.z, q.w};
}

Quaternion xRotation(double degrees) {
  const double half = radians(degrees) * 0.5;
  return {std::sin(half), 0.0, 0.0, std::cos(half)};
}

Quaternion zRotation(double degrees) {
  const double half = radians(degrees) * 0.5;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
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

Quaternion expoLandscapeToWorldToCamera(
    const Quaternion& deviceOrientation) {
  // Expo DeviceMotion's landscape quaternion and Cumquat's ENU camera basis
  // agree on the X rotation that establishes the level landscape camera, but
  // express horizontal rotation with opposite Y/Z signs. Conjugating the
  // complete quaternion also negates X and reverses pitch. Convert only the
  // two basis components involved in landscape yaw.
  return {
      deviceOrientation.x,
      -deviceOrientation.y,
      -deviceOrientation.z,
      deviceOrientation.w,
  };
}

} // namespace

Quaternion geographicallyAlignedOrientation(
    const Quaternion& currentOrientation,
    const Quaternion& initialOrientation,
    double initialHeadingDegrees) {
  // DeviceMotion supplies a stable relative pose but no dependable geographic
  // zero. Capture one synchronized heading/quaternion pair, map subsequent
  // relative rotations into Cumquat's landscape camera basis, and never use
  // later compass readings for projection.
  const Quaternion relative = multiply(
      normalized(currentOrientation),
      inverse(initialOrientation));
  const Quaternion basis = zRotation(90.0);
  const Quaternion mappedRelative = multiply(
      multiply(basis, relative),
      inverse(basis));
  const Quaternion initialCamera = multiply(
      xRotation(-90.0),
      zRotation(-initialHeadingDegrees));
  return normalized(multiply(mappedRelative, initialCamera));
}

Quaternion geographicallyAlignedGameOrientation(
    const Quaternion& currentOrientation,
    const Quaternion& initialOrientation,
    double initialHeadingDegrees) {
  // Android's game rotation vector is gravity/gyro based and intentionally has
  // no geographic north. Its relative world yaw is around Android world Z.
  // Map that world basis into Cumquat's level landscape camera basis, then
  // anchor it once using the synchronized startup compass heading.
  const Quaternion relative = multiply(
      normalized(currentOrientation),
      inverse(initialOrientation));
  const Quaternion basis = xRotation(-90.0);
  const Quaternion mappedRelative = multiply(
      multiply(basis, relative),
      inverse(basis));
  const Quaternion initialCamera = multiply(
      xRotation(-90.0),
      zRotation(-initialHeadingDegrees));
  return normalized(multiply(mappedRelative, initialCamera));
}

Vec3 worldToCamera(const Vec3& enu, const SensorState& sensorState) {
  if (sensorState.hasOrientationQuaternion) {
    if (sensorState.orientationIsEarthFromDevice) {
      // The native Android sensor supplies device -> magnetic ENU. Transform
      // the POI into physical device axes, then infer which landscape edge is
      // screen-up from gravity. This avoids Euler angles and supports both
      // landscape directions while keeping rear-camera forward fixed at -Z.
      const Quaternion deviceFromEarth = inverse(sensorState.orientation);
      const Vec3 device = rotateByQuaternion(enu, deviceFromEarth);
      const Vec3 upDevice = rotateByQuaternion(
          Vec3{0.0, 0.0, 1.0},
          deviceFromEarth);

      const bool positiveXIsScreenUp = upDevice.x >= 0.0;
      const double screenRight = positiveXIsScreenUp
          ? -device.y
          : device.y;
      const double screenUp = positiveXIsScreenUp
          ? device.x
          : -device.x;
      const double forwardDepth = -device.z;

      // Preserve projectToScreen's established quaternion camera convention:
      // right=-Y, up=-X and forward=-Z.
      return {-screenUp, -screenRight, -forwardDepth};
    }

    return rotateByQuaternion(
        enu,
        expoLandscapeToWorldToCamera(sensorState.orientation));
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

    const double focal =
        width / (2.0 * std::tan(radians(viewState.horizontalFovDeg) * 0.5));
    const double cameraRight = -camera.y;
    const double cameraUp = -camera.x;
    const double projectionDepth = forwardDepth > 0.1
        ? forwardDepth
        : std::max(camera.length(), 0.1);

    x = width * 0.5 + cameraRight * focal / projectionDepth;
    y = height * 0.5 - cameraUp * focal / projectionDepth;

    return forwardDepth > 0.1 &&
        x >= 0.0 && x <= width &&
        y >= 0.0 && y <= height;
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
