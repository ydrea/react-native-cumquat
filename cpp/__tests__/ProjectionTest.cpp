#include "../projection/Projection.h"
#include "../core/Engine.h"
#include "../android/RotationAlignment.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

using cumquat::Quaternion;
using cumquat::Engine;
using cumquat::OrientationConvention;
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

Quaternion inverse(const Quaternion& source) {
  const Quaternion normalized =
      cumquat::android::normalizedQuaternion(source);
  return {
      -normalized.x,
      -normalized.y,
      -normalized.z,
      normalized.w,
  };
}

Quaternion negated(const Quaternion& source) {
  return {-source.x, -source.y, -source.z, -source.w};
}

Quaternion quaternionFromRotationMatrix(const double matrix[3][3]) {
  Quaternion result;
  const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    result.w = 0.25 * scale;
    result.x = (matrix[2][1] - matrix[1][2]) / scale;
    result.y = (matrix[0][2] - matrix[2][0]) / scale;
    result.z = (matrix[1][0] - matrix[0][1]) / scale;
  } else if (matrix[0][0] > matrix[1][1] &&
             matrix[0][0] > matrix[2][2]) {
    const double scale =
        std::sqrt(1.0 + matrix[0][0] -
                  matrix[1][1] - matrix[2][2]) * 2.0;
    result.w = (matrix[2][1] - matrix[1][2]) / scale;
    result.x = 0.25 * scale;
    result.y = (matrix[0][1] + matrix[1][0]) / scale;
    result.z = (matrix[0][2] + matrix[2][0]) / scale;
  } else if (matrix[1][1] > matrix[2][2]) {
    const double scale =
        std::sqrt(1.0 + matrix[1][1] -
                  matrix[0][0] - matrix[2][2]) * 2.0;
    result.w = (matrix[0][2] - matrix[2][0]) / scale;
    result.x = (matrix[0][1] + matrix[1][0]) / scale;
    result.y = 0.25 * scale;
    result.z = (matrix[1][2] + matrix[2][1]) / scale;
  } else {
    const double scale =
        std::sqrt(1.0 + matrix[2][2] -
                  matrix[0][0] - matrix[1][1]) * 2.0;
    result.w = (matrix[1][0] - matrix[0][1]) / scale;
    result.x = (matrix[0][2] + matrix[2][0]) / scale;
    result.y = (matrix[1][2] + matrix[2][1]) / scale;
    result.z = 0.25 * scale;
  }
  return cumquat::android::normalizedQuaternion(result);
}

Quaternion northFacingEarthFromDevice(bool positiveXIsScreenUp) {
  if (positiveXIsScreenUp) {
    // Device +X -> up, +Y -> west, +Z -> south.
    const double matrix[3][3]{
        {0.0, -1.0, 0.0},
        {0.0, 0.0, -1.0},
        {1.0, 0.0, 0.0},
    };
    return quaternionFromRotationMatrix(matrix);
  }

  // Device +X -> down, +Y -> east, +Z -> south.
  const double matrix[3][3]{
      {0.0, 1.0, 0.0},
      {0.0, 0.0, -1.0},
      {-1.0, 0.0, 0.0},
  };
  return quaternionFromRotationMatrix(matrix);
}

void expectQuaternionEquivalent(
    const Quaternion& actual,
    const Quaternion& expected,
    double tolerance,
    std::string_view message) {
  const double error =
      cumquat::android::quaternionAngularDistanceDeg(actual, expected);
  if (error > tolerance) {
    std::cerr << "FAIL: " << message << " (angular error "
              << error << " degrees)\n";
    std::exit(EXIT_FAILURE);
  }
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

void testFullQuaternionAlignmentUsesAbsoluteSensorOnlyAtStartup() {
  cumquat::android::RotationAlignment alignment;
  const Quaternion absoluteNorth = northFacingEarthFromDevice(true);
  const Quaternion gameNorth = zRotation(37.0);

  for (std::size_t index = 0;
       index < cumquat::android::RotationAlignment::kRequiredSamples;
       ++index) {
    const std::int64_t timestamp =
        1'000'000'000 + static_cast<std::int64_t>(index) * 16'000'000;
    alignment.addPair(
        {
            index % 2 == 0 ? absoluteNorth : negated(absoluteNorth),
            timestamp,
        },
        {gameNorth, timestamp + 2'000'000},
        47.0);
  }

  expectTrue(
      alignment.isCalibrated(),
      "stable absolute/game pairs must calibrate");
  expectQuaternionEquivalent(
      alignment.alignGameOrientation(gameNorth),
      absoluteNorth,
      1e-7,
      "frozen alignment must reproduce startup Earth orientation");

  const Quaternion gameEast = multiply(
      inverse(absoluteNorth),
      multiply(zRotation(-90.0), absoluteNorth));
  const Quaternion currentGame = multiply(gameNorth, gameEast);
  const Quaternion expectedEast = multiply(
      zRotation(-90.0),
      absoluteNorth);

  expectQuaternionEquivalent(
      alignment.alignGameOrientation(currentGame),
      expectedEast,
      1e-7,
      "later orientation must depend only on the game quaternion");

  // Once calibrated, even an absurd later absolute sample is ignored.
  alignment.addPair(
      {zRotation(180.0), 9'000'000'000},
      {currentGame, 9'000'000'000},
      47.0);
  expectQuaternionEquivalent(
      alignment.alignGameOrientation(currentGame),
      expectedEast,
      1e-7,
      "later magnetometer orientation must not alter frozen alignment");
}

void testAlignmentRejectsBadFieldAndUnsynchronizedPairs() {
  cumquat::android::RotationAlignment alignment;
  const Quaternion absolute = northFacingEarthFromDevice(true);
  const Quaternion game{};

  for (std::size_t index = 0;
       index < cumquat::android::RotationAlignment::kRequiredSamples;
       ++index) {
    const std::int64_t timestamp =
        1'000'000'000 + static_cast<std::int64_t>(index) * 16'000'000;
    alignment.addPair(
        {absolute, timestamp},
        {game, timestamp},
        7.0);
  }
  expectFalse(
      alignment.isCalibrated(),
      "implausible magnetic field must not establish alignment");

  for (std::size_t index = 0;
       index < cumquat::android::RotationAlignment::kRequiredSamples;
       ++index) {
    const std::int64_t timestamp =
        2'000'000'000 + static_cast<std::int64_t>(index) * 16'000'000;
    alignment.addPair(
        {absolute, timestamp},
        {game, timestamp + 100'000'000},
        47.0);
  }
  expectFalse(
      alignment.isCalibrated(),
      "samples separated by 100 ms must not establish alignment");
}

void testEarthAlignedProjectionSupportsBothLandscapeDirections() {
  for (const bool positiveXIsScreenUp : {true, false}) {
    SensorState sensor = quaternionSensor();
    sensor.orientationIsEarthFromDevice = true;
    sensor.orientation =
        northFacingEarthFromDevice(positiveXIsScreenUp);

    const Vec3 northCamera =
        cumquat::projection::worldToCamera({0.0, 1.0, 0.0}, sensor);
    expectNear(
        northCamera.z,
        -1.0,
        "north-facing rear camera must see north in either landscape side");

    const Vec3 eastCamera =
        cumquat::projection::worldToCamera({1.0, 0.0, 0.0}, sensor);
    expectTrue(
        -eastCamera.y > 0.0,
        "east must appear screen-right while camera faces north");

    const Vec3 upCamera =
        cumquat::projection::worldToCamera({0.0, 0.0, 1.0}, sensor);
    expectTrue(
        -upCamera.x > 0.0,
        "Earth up must appear screen-up in either landscape side");
  }
}

void testEarthAlignedEngineNeverUsesScalarHeading() {
  Engine engine;
  engine.initialize({
      POI{"north", "North", {0.001, 0.0, 0.0}},
  });

  SensorState sensor = quaternionSensor();
  sensor.location = {0.0, 0.0, 0.0};
  sensor.usesGameRotationVector = true;
  sensor.orientationIsEarthFromDevice = true;
  sensor.orientation = northFacingEarthFromDevice(true);
  sensor.headingDeg = 217.0;

  engine.update(sensor);
  expectTrue(
      engine.getFrame().projectedPOIs.size() == 1,
      "full Earth alignment must not wait for scalar heading");
  const auto calibrated = engine.getFrame().projectedPOIs.front();

  sensor.headingDeg = 0.0;
  sensor.initialHeadingDeg = 359.0;
  sensor.hasInitialHeading = true;
  engine.update(sensor);
  const auto afterCompassJump = engine.getFrame().projectedPOIs.front();

  expectNear(
      afterCompassJump.x,
      calibrated.x,
      "scalar heading changes must not move Earth-aligned X");
  expectNear(
      afterCompassJump.y,
      calibrated.y,
      "scalar heading changes must not move Earth-aligned Y");
  expectNear(
      afterCompassJump.depth,
      calibrated.depth,
      "scalar heading changes must not move Earth-aligned depth");
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

void testGameRotationVectorCardinalTurnsKeepGeographicAlignment() {
  const Quaternion initial{0.0, 0.0, 0.0, 1.0};

  const auto expectGameForward = [&](
      const Quaternion& current,
      double poiBearing,
      std::string_view message) {
    SensorState sensor = quaternionSensor();
    sensor.orientation =
        cumquat::projection::geographicallyAlignedGameOrientation(
            current, initial, 0.0);
    const Vec3 camera = cumquat::projection::worldToCamera(
        horizontalVector(poiBearing), sensor);

    expectNear(camera.x, 0.0, message);
    expectNear(camera.y, 0.0, message);
    expectNear(camera.z, -1.0, message);
  };

  // Android world Z is positive counter-clockwise. Turning the camera
  // clockwise from north to east therefore produces a -90° relative rotation.
  expectGameForward(initial, 0.0, "game-vector north must face north");
  expectGameForward(
      zRotation(-90.0), 90.0, "game-vector east must face east");
  expectGameForward(
      zRotation(180.0), 180.0, "game-vector south must face south");
  expectGameForward(
      zRotation(90.0), 270.0, "game-vector west must face west");
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

void testEngineNeverFallsBackWhileAwaitingGameRotationVector() {
  Engine engine;
  engine.initialize({
      POI{"north", "North", {0.001, 0.0, 0.0}},
  });

  SensorState sensor;
  sensor.location = {0.0, 0.0, 0.0};
  sensor.viewportWidth = 100.0;
  sensor.viewportHeight = 100.0;
  sensor.headingDeg = 180.0;
  sensor.initialHeadingDeg = 180.0;
  sensor.hasInitialHeading = true;
  sensor.usesGameRotationVector = true;
  sensor.hasOrientationQuaternion = false;

  engine.update(sensor);
  expectTrue(
      engine.getFrame().projectedPOIs.empty(),
      "Android must not fall back to magnetometer heading before game-vector data");
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

void testFrameExposesEarthFromDeviceOrientation() {
  Engine engine;
  engine.initialize(std::vector<POI>{});

  SensorState sensor = quaternionSensor();
  sensor.timestampNs = 42;
  sensor.orientation = {0.1, 0.2, 0.3, 0.9};
  sensor.orientationIsEarthFromDevice = true;

  engine.update(sensor);
  const auto& orientation = engine.getFrame().orientation;
  expectTrue(orientation.has_value(), "ready frame should expose orientation");
  expectTrue(
      orientation->convention == OrientationConvention::EarthFromDevice,
      "native Android orientation must declare earth-from-device convention");
  expectNear(orientation->quaternion.x, 0.1, "orientation X should be preserved");
  expectNear(orientation->quaternion.y, 0.2, "orientation Y should be preserved");
  expectNear(orientation->quaternion.z, 0.3, "orientation Z should be preserved");
  expectNear(orientation->quaternion.w, 0.9, "orientation W should be preserved");
}

void testFrameOrientationIsNullUntilReferenceIsReady() {
  Engine engine;
  engine.initialize(std::vector<POI>{});

  SensorState sensor = quaternionSensor();
  engine.update(sensor);

  expectFalse(
      engine.getFrame().orientation.has_value(),
      "unaligned DeviceMotion frame must not expose an invented orientation");
}

} // namespace

int main() {
  testFullQuaternionAlignmentUsesAbsoluteSensorOnlyAtStartup();
  testAlignmentRejectsBadFieldAndUnsynchronizedPairs();
  testEarthAlignedProjectionSupportsBothLandscapeDirections();
  testEarthAlignedEngineNeverUsesScalarHeading();
  testExpoLandscapeYawIsConvertedToCameraDirection();
  testCapturedGalaxyS7CardinalTurnsKeepGeographicAlignment();
  testInitialHeadingAnchorsSameQuaternionToGeographicDirection();
  testGameRotationVectorCardinalTurnsKeepGeographicAlignment();
  testEngineConsumesInitialHeadingOnlyOnce();
  testEngineNeverFallsBackWhileAwaitingGameRotationVector();
  testHorizontalYawDirectionIsReversedWithoutFullConjugation();
  testLandscapePitchAxisIsNotConjugated();
  testIdentityQuaternionPreservesWorldVector();
  testPerspectiveUsesForwardCameraDepth();
  testCloserPointHasLargerPerspectiveDisplacement();
  testBehindCameraIsRejected();
  testBehindCameraRetainsDirectionalCoordinates();
  testStrictViewportBoundsRejectOverscan();
  testVerticalProjectionUsesSquarePixelFocalLength();
  testFrameExposesEarthFromDeviceOrientation();
  testFrameOrientationIsNullUntilReferenceIsReady();
  std::cout << "All Cumquat projection tests passed\n";
  return EXIT_SUCCESS;
}
