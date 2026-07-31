#include "NativeCumquatModule.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace facebook::react {
namespace {

jsi::Value property(
    jsi::Runtime& runtime,
    const jsi::Object& object,
    const char* name) {
  return object.getProperty(runtime, name);
}

double numberOr(
    jsi::Runtime& runtime,
    const jsi::Object& object,
    const char* name,
    double fallback) {
  const auto value = property(runtime, object, name);
  return value.isNumber() ? value.asNumber() : fallback;
}

std::string stringOr(
    jsi::Runtime& runtime,
    const jsi::Object& object,
    const char* name,
    std::string fallback = {}) {
  const auto value = property(runtime, object, name);
  return value.isString() ? value.asString(runtime).utf8(runtime)
                          : std::move(fallback);
}

jsi::Object requiredObject(
    jsi::Runtime& runtime,
    const jsi::Object& object,
    const char* name) {
  auto value = property(runtime, object, name);
  if (!value.isObject()) {
    throw jsi::JSError(
        runtime,
        std::string("Cumquat expected object property '") + name + "'");
  }
  return value.asObject(runtime);
}

cumquat::GeoPoint readGeoPoint(
    jsi::Runtime& runtime,
    const jsi::Object& object) {
  return {
      numberOr(runtime, object, "latitude", 0.0),
      numberOr(runtime, object, "longitude", 0.0),
      numberOr(runtime, object, "altitude", 0.0),
  };
}

void validateFinite(
    jsi::Runtime& runtime,
    double value,
    const char* propertyName) {
  if (!std::isfinite(value)) {
    throw jsi::JSError(
        runtime,
        std::string("Cumquat expected finite '") + propertyName + "'");
  }
}

cumquat::ViewState readViewState(
    jsi::Runtime& runtime,
    const jsi::Object& object,
    const cumquat::ViewState& fallback = {}) {
  cumquat::ViewState viewState{
      numberOr(
          runtime,
          object,
          "horizontalFovDegrees",
          fallback.horizontalFovDeg),
      numberOr(
          runtime,
          object,
          "minDistanceMeters",
          fallback.minDistanceMeters),
      numberOr(
          runtime,
          object,
          "maxDistanceMeters",
          fallback.maxDistanceMeters),
  };

  validateFinite(runtime, viewState.horizontalFovDeg, "horizontalFovDegrees");
  validateFinite(runtime, viewState.minDistanceMeters, "minDistanceMeters");
  validateFinite(runtime, viewState.maxDistanceMeters, "maxDistanceMeters");

  if (viewState.horizontalFovDeg <= 1.0 ||
      viewState.horizontalFovDeg >= 179.0) {
    throw jsi::JSError(
        runtime,
        "Cumquat horizontalFovDegrees must be between 1 and 179");
  }
  if (viewState.minDistanceMeters < 0.0 ||
      viewState.maxDistanceMeters <= viewState.minDistanceMeters) {
    throw jsi::JSError(runtime, "Cumquat invalid view-state distance range");
  }

  return viewState;
}

jsi::Object serializeProjectedPOI(
    jsi::Runtime& runtime,
    const cumquat::VisiblePOI& source) {
  jsi::Object output(runtime);
  output.setProperty(runtime, "poiIndex", static_cast<double>(source.poiIndex));
  output.setProperty(runtime, "x", source.x);
  output.setProperty(runtime, "y", source.y);
  output.setProperty(runtime, "depth", source.depth);
  output.setProperty(runtime, "distance", source.distance);
  output.setProperty(runtime, "bearing", source.bearingDeg);
  output.setProperty(runtime, "visible", source.visible);
  output.setProperty(runtime, "clipped", source.clipped);

  switch (source.clippedByDistance) {
    case cumquat::DistanceClip::Min:
      output.setProperty(
          runtime,
          "clippedByDistance",
          jsi::String::createFromUtf8(runtime, "min"));
      break;
    case cumquat::DistanceClip::Max:
      output.setProperty(
          runtime,
          "clippedByDistance",
          jsi::String::createFromUtf8(runtime, "max"));
      break;
    case cumquat::DistanceClip::None:
    default:
      output.setProperty(runtime, "clippedByDistance", jsi::Value::null());
      break;
  }

  return output;
}

} // namespace

NativeCumquatModule::NativeCumquatModule(
    std::shared_ptr<CallInvoker> jsInvoker)
    : NativeCumquatCxxSpec(std::move(jsInvoker)) {}

NativeCumquatModule::~NativeCumquatModule() = default;

std::string NativeCumquatModule::getVersion(jsi::Runtime&) {
  return "0.1.0-cpp";
}

double NativeCumquatModule::createEngine(
    jsi::Runtime& runtime,
    jsi::Object configObject) {
  cumquat::EngineConfig config;

  const double legacyFarMeters =
      numberOr(runtime, configObject, "farMeters", config.datasetRadiusMeters);
  config.datasetRadiusMeters = numberOr(
      runtime,
      configObject,
      "datasetRadiusMeters",
      legacyFarMeters);

  const auto maxVisible = numberOr(
      runtime,
      configObject,
      "maxVisiblePOIs",
      static_cast<double>(config.maxVisiblePOIs));

  validateFinite(runtime, config.datasetRadiusMeters, "datasetRadiusMeters");
  validateFinite(runtime, maxVisible, "maxVisiblePOIs");

  if (config.datasetRadiusMeters <= 0.0) {
    throw jsi::JSError(runtime, "Cumquat datasetRadiusMeters must be positive");
  }

  config.maxVisiblePOIs = static_cast<std::uint32_t>(std::clamp(
      maxVisible,
      1.0,
      static_cast<double>(std::numeric_limits<std::uint32_t>::max())));

  auto engine = std::make_shared<cumquat::Engine>(config);

  cumquat::ViewState initialViewState;
  initialViewState.horizontalFovDeg = numberOr(
      runtime,
      configObject,
      "horizontalFovDegrees",
      initialViewState.horizontalFovDeg);
  initialViewState.minDistanceMeters = numberOr(
      runtime,
      configObject,
      "nearMeters",
      initialViewState.minDistanceMeters);
  initialViewState.maxDistanceMeters = legacyFarMeters;
  engine->setViewState(initialViewState);

  std::lock_guard lock(mutex_);
  const auto handle = nextHandle_++;
  engines_.emplace(handle, std::move(engine));
  return static_cast<double>(handle);
}

void NativeCumquatModule::initialize(
    jsi::Runtime& runtime,
    double handle,
    jsi::Array poiArray) {
  auto engine = requireEngine(runtime, handle);
  const auto count = poiArray.size(runtime);

  std::vector<cumquat::POI> pois;
  pois.reserve(count);

  for (std::size_t index = 0; index < count; ++index) {
    auto value = poiArray.getValueAtIndex(runtime, index);
    if (!value.isObject()) {
      throw jsi::JSError(runtime, "Cumquat POIs must be objects");
    }

    auto object = value.asObject(runtime);
    cumquat::POI poi;
    poi.id = stringOr(runtime, object, "id");
    poi.name = stringOr(runtime, object, "name");
    poi.position = readGeoPoint(runtime, object);

    if (poi.id.empty()) {
      throw jsi::JSError(runtime, "Cumquat POI id cannot be empty");
    }

    validateFinite(runtime, poi.position.latitudeDeg, "latitude");
    validateFinite(runtime, poi.position.longitudeDeg, "longitude");
    validateFinite(runtime, poi.position.altitudeMeters, "altitude");
    pois.push_back(std::move(poi));
  }

  engine->initialize(std::move(pois));
}

void NativeCumquatModule::setViewState(
    jsi::Runtime& runtime,
    double handle,
    jsi::Object viewStateObject) {
  auto engine = requireEngine(runtime, handle);
  engine->setViewState(
      readViewState(runtime, viewStateObject, engine->getViewState()));
}

double NativeCumquatModule::update(
    jsi::Runtime& runtime,
    double handle,
    jsi::Object sensorObject) {
  auto engine = requireEngine(runtime, handle);

  cumquat::SensorState sensorState;
  sensorState.timestampNs = static_cast<std::int64_t>(
      numberOr(runtime, sensorObject, "timestampNs", 0.0));
  sensorState.location = readGeoPoint(
      runtime,
      requiredObject(runtime, sensorObject, "location"));
  sensorState.headingDeg =
      numberOr(runtime, sensorObject, "headingDegrees", 0.0);
  auto initialHeadingValue =
      property(runtime, sensorObject, "initialHeadingDegrees");
  if (initialHeadingValue.isNumber()) {
    sensorState.initialHeadingDeg = initialHeadingValue.asNumber();
    sensorState.hasInitialHeading = true;
    validateFinite(
        runtime,
        sensorState.initialHeadingDeg,
        "initialHeadingDegrees");
  }
  sensorState.pitchDeg =
      numberOr(runtime, sensorObject, "pitchDegrees", 0.0);
  sensorState.rollDeg =
      numberOr(runtime, sensorObject, "rollDegrees", 0.0);
  sensorState.viewportWidth =
      numberOr(runtime, sensorObject, "viewportWidth", 1.0);
  sensorState.viewportHeight =
      numberOr(runtime, sensorObject, "viewportHeight", 1.0);

  auto orientationValue = property(runtime, sensorObject, "orientationQuaternion");
  if (orientationValue.isObject()) {
    auto orientationObject = orientationValue.asObject(runtime);
    sensorState.orientation = {
        numberOr(runtime, orientationObject, "x", 0.0),
        numberOr(runtime, orientationObject, "y", 0.0),
        numberOr(runtime, orientationObject, "z", 0.0),
        numberOr(runtime, orientationObject, "w", 1.0),
    };

    validateFinite(runtime, sensorState.orientation.x, "orientationQuaternion.x");
    validateFinite(runtime, sensorState.orientation.y, "orientationQuaternion.y");
    validateFinite(runtime, sensorState.orientation.z, "orientationQuaternion.z");
    validateFinite(runtime, sensorState.orientation.w, "orientationQuaternion.w");

    const double quaternionNorm = std::sqrt(
        sensorState.orientation.x * sensorState.orientation.x +
        sensorState.orientation.y * sensorState.orientation.y +
        sensorState.orientation.z * sensorState.orientation.z +
        sensorState.orientation.w * sensorState.orientation.w);
    if (quaternionNorm <= std::numeric_limits<double>::epsilon()) {
      throw jsi::JSError(
          runtime,
          "Cumquat orientation quaternion cannot have zero length");
    }
    sensorState.hasOrientationQuaternion = true;
  }

#ifdef __ANDROID__
  // Expo DeviceMotion uses Android's normal rotation vector, which may contain
  // continuous geomagnetic corrections. Never let that fused orientation
  // become either the initial reference or a later projection update.
  sensorState.hasOrientationQuaternion = false;
  sensorState.usesGameRotationVector = true;

  cumquat::Quaternion gameOrientation;
  if (gameRotationSensor_.latest(gameOrientation)) {
    sensorState.orientation = gameOrientation;
    sensorState.hasOrientationQuaternion = true;
    sensorState.orientationIsEarthFromDevice = true;
  }
#endif

  validateFinite(runtime, sensorState.location.latitudeDeg, "location.latitude");
  validateFinite(runtime, sensorState.location.longitudeDeg, "location.longitude");
  validateFinite(runtime, sensorState.location.altitudeMeters, "location.altitude");
  validateFinite(runtime, sensorState.headingDeg, "headingDegrees");
  validateFinite(runtime, sensorState.pitchDeg, "pitchDegrees");
  validateFinite(runtime, sensorState.rollDeg, "rollDegrees");
  validateFinite(runtime, sensorState.viewportWidth, "viewportWidth");
  validateFinite(runtime, sensorState.viewportHeight, "viewportHeight");

  if (sensorState.viewportWidth <= 0.0 || sensorState.viewportHeight <= 0.0) {
    throw jsi::JSError(runtime, "Cumquat viewport must be positive");
  }

  return static_cast<double>(engine->update(sensorState));
}

jsi::Object NativeCumquatModule::getFrame(
    jsi::Runtime& runtime,
    double handle) {
  const auto engine = requireEngine(runtime, handle);
  const auto& snapshot = engine->getFrame();

  jsi::Object frame(runtime);
  frame.setProperty(runtime, "sequence", static_cast<double>(snapshot.sequence));
  frame.setProperty(
      runtime,
      "timestampNs",
      static_cast<double>(snapshot.timestampNs));

  jsi::Array projected(runtime, snapshot.projectedPOIs.size());
  for (std::size_t index = 0; index < snapshot.projectedPOIs.size(); ++index) {
    projected.setValueAtIndex(
        runtime,
        index,
        serializeProjectedPOI(runtime, snapshot.projectedPOIs[index]));
  }
  frame.setProperty(runtime, "projectedPOIs", std::move(projected));

  jsi::Array visible(runtime, snapshot.visiblePOIs.size());
  for (std::size_t index = 0; index < snapshot.visiblePOIs.size(); ++index) {
    visible.setValueAtIndex(
        runtime,
        index,
        serializeProjectedPOI(runtime, snapshot.visiblePOIs[index]));
  }
  frame.setProperty(runtime, "visiblePOIs", std::move(visible));

  return frame;
}

std::optional<jsi::Object> NativeCumquatModule::pick(
    jsi::Runtime& runtime,
    double handle,
    double x,
    double y,
    double radiusPixels) {
  validateFinite(runtime, x, "x");
  validateFinite(runtime, y, "y");
  validateFinite(runtime, radiusPixels, "radiusPixels");

  const auto engine = requireEngine(runtime, handle);
  const auto result = engine->pick(x, y, std::max(0.0, radiusPixels));
  if (!result.has_value()) {
    return std::nullopt;
  }

  jsi::Object output(runtime);
  output.setProperty(
      runtime,
      "poiIndex",
      static_cast<double>(result->poiIndex));
  output.setProperty(runtime, "distancePixels", result->distancePixels);
  return output;
}

void NativeCumquatModule::destroyEngine(jsi::Runtime& runtime, double handle) {
  const auto key = validateHandle(runtime, handle);
  std::lock_guard lock(mutex_);
  engines_.erase(key);
}

NativeCumquatModule::EnginePtr NativeCumquatModule::requireEngine(
    jsi::Runtime& runtime,
    double handle) const {
  const auto key = validateHandle(runtime, handle);
  std::lock_guard lock(mutex_);
  const auto iterator = engines_.find(key);
  if (iterator == engines_.end()) {
    throw jsi::JSError(runtime, "Cumquat engine handle is invalid or disposed");
  }
  return iterator->second;
}

std::uint64_t NativeCumquatModule::validateHandle(
    jsi::Runtime& runtime,
    double handle) {
  if (!std::isfinite(handle) || handle < 1.0 || std::floor(handle) != handle) {
    throw jsi::JSError(runtime, "Cumquat received an invalid engine handle");
  }
  return static_cast<std::uint64_t>(handle);
}

} // namespace facebook::react
