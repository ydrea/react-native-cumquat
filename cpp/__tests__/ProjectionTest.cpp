#include "../projection/Projection.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {

using cumquat::Quaternion;
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

void testPreparedWorldToCameraQuaternionIsAppliedDirectly() {
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
      1.0,
      "prepared world-to-camera quaternion must not be conjugated again");
  expectNear(camera.z, 0.0, "yaw should preserve camera Z");
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
  testPreparedWorldToCameraQuaternionIsAppliedDirectly();
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
