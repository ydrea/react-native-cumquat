#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cumquat {

struct GeoPoint {
  double latitudeDeg{0.0};
  double longitudeDeg{0.0};
  double altitudeMeters{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct POI {
  std::string id;
  std::string name;
  GeoPoint position;
};

struct EngineConfig {
  double datasetRadiusMeters{135000.0};
  std::uint32_t maxVisiblePOIs{256};
};

struct ViewState {
  double horizontalFovDeg{90.0};
  double minDistanceMeters{0.0};
  double maxDistanceMeters{135000.0};
};

struct SensorState {
  std::int64_t timestampNs{0};
  GeoPoint location;
  Quaternion orientation;
  bool hasOrientationQuaternion{false};
  bool usesGameRotationVector{false};
  bool orientationIsEarthFromDevice{false};
  double initialHeadingDeg{0.0};
  bool hasInitialHeading{false};
  double headingDeg{0.0};
  double pitchDeg{0.0};
  double rollDeg{0.0};
  double viewportWidth{1.0};
  double viewportHeight{1.0};
};

enum class DistanceClip : std::uint8_t {
  None = 0,
  Min = 1,
  Max = 2,
};

struct VisiblePOI {
  std::uint32_t poiIndex{0};
  double x{0.0};
  double y{0.0};
  double depth{0.0};
  double distance{0.0};
  double bearingDeg{0.0};
  bool visible{false};
  bool clipped{true};
  DistanceClip clippedByDistance{DistanceClip::None};
};

enum class OrientationConvention : std::uint8_t {
  EarthFromDevice = 0,
  WorldToCamera = 1,
};

struct FrameOrientation {
  Quaternion quaternion;
  OrientationConvention convention{OrientationConvention::WorldToCamera};
};

struct FrameSnapshot {
  std::uint64_t sequence{0};
  std::int64_t timestampNs{0};
  std::optional<FrameOrientation> orientation;
  std::vector<VisiblePOI> projectedPOIs;
  std::vector<VisiblePOI> visiblePOIs;
};

struct PickResult {
  std::uint32_t poiIndex{0};
  double distancePixels{0.0};
};

} // namespace cumquat
