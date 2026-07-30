#include "../projection/Projection.h"
#include "../core/Engine.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

using cumquat::Quaternion;
using cumquat::Engine;
using cumquat::POI;
using cumquat::SensorState;
using cumquat::Vec3;
using cumquat::ViewState;

constexpr double kTolerance = 1e-9;

void fail(std::string_view message) {
  std::cerr << "FAIL: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

void expectTrue(bool value, std::string_view message) {
  if (!value) fail(message);
}

void expectFalse(bool value, std::string_view message) {
  if (value) fail(message);
}

void expectNear(
    double actual,
    double expected,
    std::string_view message) {
  if (std::abs(actual - expected) > kTolerance) {
    std::cerr << "FAIL: " << message << " (expected " << expected
              << ", received " << actual << ")\n";
    std::exit(EXIT_FAILURE);
  }
}

void expectNearWithTolerance(
    double actual,
    double expected,
    double tolerance,
    std::string_view message) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "FAIL: " << message << " (expected " << expected
              << " ± " << tolerance << ", received " << actual << ")\n";
    std::exit(EXIT_FAILURE);
  }
}

SensorState quaternionSensor(double width = 100.0, double height = 100.0) {
  SensorState sensor;
  sensor.hasOrientationQuaternion = true;
  sensor.viewportWidth = width;
  sensor.viewportHeight = height;
  return sensor;
}

ViewState ninetyDegreeView() {
  ViewState view;
  view.horizontalFovDeg = 90.0;
  return view;
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

Quaternion xRotation(double degrees) {
  const double halfRadians = degrees * 3.14159265358979323846 / 360.0;
  return {
      std::sin(halfRadians),
      0.0,
      0.0,
      std::cos(halfRadians),
  };
}

Quaternion zRotation(double degrees) {
  const double halfRadians = degrees * 3.14159265358979323846 / 360.0;
  return {
      0.0,
      0.0,
      std::sin(halfRadians),
      std::cos(halfRadians),
  };
}

Vec3 horizontalVector(double bearingDegrees) {
  const double radians =
      bearingDegrees * 3.14159265358979323846 / 180.0;
  return {
      std::sin(radians),
      std::cos(radians),
      0.0,
  };
}

Quaternion expoLandscapeOrientation(double bearingDegrees) {
  // Expo's horizontal yaw direction is opposite Cumquat's ENU camera
  // direction after the level landscape X rotation.
  return multiply(xRotation(-90.0), zRotation(-bearingDegrees));
}

void expectForward(
    double bearingDegrees,
    std::string_view message) {
  SensorState sensor = quaternionSensor();
  sensor.orientation = expoLandscapeOrientation(bearingDegrees);

  const Vec3 camera = cumquat::projection::worldToCamera(
      horizontalVector(bearingDegrees),
      sensor);

  expectNear(camera.x, 0.0, message);
  expectNear(camera.y, 0.0, message);
  expectNear(camera.z, -1.0, message);
}

void testExpoLandscapeYawIsConvertedToCameraDirection() {
  expectForward(0.0, "north must map to camera forward");
  expectForward(90.0, "east must map to camera forward");
  expectForward(180.0, "south must map to camera forward");
  expectForward(270.0, "west must map to camera forward");
}

void testCapturedGalaxyS7CardinalTurnsKeepGeographicAlignment() {
  // Real screen-orientation=90 samples captured on the Galaxy S7. The
  // DeviceMotion deltas are ~83° north→west and ~88° west→south even though
  // the previous component-sign conversion collapsed them onto one direction.
  const Quaternion north{
      -0.023453, -0.863483, -0.502306, -0.039194};
  const Quaternion west{
      0.012318, -0.318339, -0.945196, 0.071508};
  const Quaternion south{
      -0.047090, 0.415941, -0.905118, -0.074410};

  const auto expectCapturedForward = [&](
      const Quaternion& current,
      double poiBearing,
      std::string_view message) {
    SensorState sensor = quaternionSensor();
    sensor.orientation =
        cumquat::projection::geographicallyAlignedOrientation(
            current, north, 0.0);
    const Vec3 camera = cumquat::projection::worldToCamera(
        horizontalVector(poiBearing), sensor);

    expectNearWithTolerance(camera.x, 0.0, 0.25, message);
    expectNearWithTolerance(camera.y, 0.0, 0.25, message);
    expectNearWithTolerance(camera.z, -1.0, 0.05, message);
  };

  expectCapturedForward(north, 0.0, "captured north must face north");
  expectCapturedForward(west, 270.0, "captured west must face west");
  expectCapturedForward(south, 180.0, "captured south must face south");
}

void testInitialHeadingAnchorsSameQuaternionToGeographicDirection() {
  const Quaternion initial{
      -0.023453, -0.863483, -0.502306, -0.039194};

  for (const double heading : {0.0, 90.0, 180.0, 270.0}) {
    SensorState sensor = quaternionSensor();
    sensor.orientation =
        cumquat::projection::geographicallyAlignedOrientation(
            initial, initial, heading);
    const Vec3 camera = cumquat::projection::worldToCamera(
        horizontalVector(heading), sensor);

    expectNearWithTolerance(
        camera.z,
        -1.0,
        1e-9,
        "initial heading must establish geographic camera forward");
  }
}

void testEngineConsumesInitialHeadingOnlyOnce() {
  const Quaternion north{
      -0.023453, -0.863483, -0.502306, -0.039194};
  Engine engine;
  engine.initialize({
      POI{"north", "North", {0.001, 0.0, 0.0}},
  });

  SensorState sensor = quaternionSensor();
  sensor.location = {0.0, 0.0, 0.0};
  sensor.orientation = north;
  sensor.headingDeg = 217.0;

  engine.update(sensor);
  expectTrue(
      engine.getFrame().projectedPOIs.empty(),
      "engine must wait for an explicit synchronized initial heading");

  sensor.hasInitialHeading = true;
  sensor.initialHeadingDeg = 0.0;
  engine.update(sensor);
  expectTrue(
      engine.getFrame().projectedPOIs.size() == 1,
      "initial heading/quaternion pair must start projection");
  const auto calibrated = engine.getFrame().projectedPOIs.front();

  sensor.hasInitialHeading = false;
  sensor.headingDeg = 359.0;
  engine.update(sensor);
  const auto afterCompassJump = engine.getFrame().projectedPOIs.front();

  expectNear(
      afterCompassJump.x,
      calibrated.x,
      "later compass changes must not move projection X");
  expectNear(
      afterCompassJump.y,
      calibrated.y,
      "later compass changes must not move projection Y");
  expectNear(
      afterCompassJump.depth,
      calibrated.depth,
      "later compass changes must not move projection depth");
}

void testHorizontalYawDirectionIsReversedWithoutFullConjugation() {
  constexpr double halfSqrtTwo = 0.70710678118654752440;
  SensorState sensor = quaternionSensor();
  sensor.orientation = Quaternion{
      0.0,
      0.0,
      halfSqrtTwo,
      halfSqrtTwo,
  };

  const Vec3 camera =
      cumquat::projection::worldToCamera(Vec3{1.0, 0.0, 0.0}, sensor);

  expectNear(camera.x, 0.0, "90-degree device yaw should remove camera X");
  expectNear(
      camera.y,
      -1.0,
      "positive horizontal yaw must rotate in the camera direction");
  expectNear(camera.z, 0.0, "yaw should preserve camera Z");
}

void testLandscapePitchAxisIsNotConjugated() {
  SensorState sensor = quaternionSensor();
  sensor.orientation = xRotation(-45.0);
  const Vec3 world{0.0, 1.0, 0.0};

  const Vec3 camera =
      cumquat::projection::worldToCamera(world, sensor);

  constexpr double halfSqrtTwo = 0.70710678118654752440;
  expectNear(camera.x, 0.0, "X-axis pitch should preserve X");
  expectNear(
      camera.y,
      halfSqrtTwo,
      "X-axis pitch must retain its original sign");
  expectNear(
      camera.z,
      -halfSqrtTwo,
      "X-axis pitch must not be reversed by full conjugation");
}

void testIdentityQuaternionPreservesWorldVector() {
  const SensorState sensor = quaternionSensor();
  const Vec3 world{3.0, -4.0, 5.0};
  const Vec3 camera = cumquat::projection::worldToCamera(world, sensor);

  expectNear(camera.x, world.x, "identity quaternion should preserve X");
  expectNear(camera.y, world.y, "identity quaternion should preserve Y");
  expectNear(camera.z, world.z, "identity quaternion should preserve Z");
}

void testPerspectiveUsesForwardCameraDepth() {
  const SensorState sensor = quaternionSensor();
  const ViewState view = ninetyDegreeView();
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;

  const bool visible = cumquat::projection::projectToScreen(
      Vec3{0.0, -5.0, -10.0}, sensor, view, x, y, depth);

  expectTrue(visible, "point inside horizontal FOV should be visible");
  expectNear(x, 75.0, "perspective X should divide by forward depth");
  expectNear(y, 50.0, "zero camera-up displacement should remain centered");
  expectNear(depth, 10.0, "reported depth should be forward camera depth");
}

void testCloserPointHasLargerPerspectiveDisplacement() {
  const SensorState sensor = quaternionSensor();
  const ViewState view = ninetyDegreeView();
  double farX = 0.0;
  double farY = 0.0;
  double farDepth = 0.0;
  double nearX = 0.0;
  double nearY = 0.0;
  double nearDepth = 0.0;

  expectTrue(
      cumquat::projection::projectToScreen(
          Vec3{0.0, -2.0, -10.0},
          sensor,
          view,
          farX,
          farY,
          farDepth),
      "far test point should be visible");
  expectTrue(
      cumquat::projection::projectToScreen(
          Vec3{0.0, -2.0, -5.0},
          sensor,
          view,
          nearX,
          nearY,
          nearDepth),
      "near test point should be visible");
  expectTrue(
      nearX > farX,
      "screen displacement should increase as forward depth decreases");
}

void testBehindCameraIsRejected() {
  const SensorState sensor = quaternionSensor();
  const ViewState view = ninetyDegreeView();
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;

  expectFalse(
      cumquat::projection::projectToScreen(
          Vec3{0.0, 0.0, 10.0}, sensor, view, x, y, depth),
      "point behind the camera must not be visible");
  expectNear(depth, -10.0, "behind-camera point should have negative depth");
}

void testBehindCameraRetainsDirectionalCoordinates() {
  const SensorState sensor = quaternionSensor();
  const ViewState view = ninetyDegreeView();
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;

  expectFalse(
      cumquat::projection::projectToScreen(
          Vec3{0.0, -5.0, 10.0}, sensor, view, x, y, depth),
      "behind-camera point must not be visible");
  expectTrue(
      x > sensor.viewportWidth * 0.5,
      "behind-camera point must retain a rightward edge direction");
  expectNear(y, 50.0, "behind-camera horizontal direction should stay centered");
}

void testStrictViewportBoundsRejectOverscan() {
  const SensorState sensor = quaternionSensor();
  const ViewState view = ninetyDegreeView();
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;

  expectFalse(
      cumquat::projection::projectToScreen(
          Vec3{0.0, -11.0, -10.0}, sensor, view, x, y, depth),
      "point beyond the horizontal viewport must not be marked visible");
  expectNear(x, 105.0, "offscreen projection should retain its screen coordinate");
}

void testVerticalProjectionUsesSquarePixelFocalLength() {
  const SensorState sensor = quaternionSensor(200.0, 100.0);
  const ViewState view = ninetyDegreeView();
  double x = 0.0;
  double y = 0.0;
  double depth = 0.0;

  expectTrue(
      cumquat::projection::projectToScreen(
          Vec3{-2.0, 0.0, -10.0}, sensor, view, x, y, depth),
      "vertical test point should be visible");
  expectNear(x, 100.0, "zero camera-right displacement should remain centered");
  expectNear(y, 30.0, "vertical projection should use the pixel focal length");
}

} // namespace

int main() {
  testExpoLandscapeYawIsConvertedToCameraDirection();
  testCapturedGalaxyS7CardinalTurnsKeepGeographicAlignment();
  testInitialHeadingAnchorsSameQuaternionToGeographicDirection();
  testEngineConsumesInitialHeadingOnlyOnce();
  testHorizontalYawDirectionIsReversedWithoutFullConjugation();
  testLandscapePitchAxisIsNotConjugated();
  testIdentityQuaternionPreservesWorldVector();
  testPerspectiveUsesForwardCameraDepth();
  testCloserPointHasLargerPerspectiveDisplacement();
  testBehindCameraIsRejected();
  testBehindCameraRetainsDirectionalCoordinates();
  testStrictViewportBoundsRejectOverscan();
  testVerticalProjectionUsesSquarePixelFocalLength();
  std::cout << "All Cumquat projection tests passed\n";
  return EXIT_SUCCESS;
}
