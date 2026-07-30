#include "Engine.h"

#include "../geo/Geodesy.h"
#include "../projection/Projection.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cumquat {

Engine::Engine(EngineConfig config) : config_(config) {
  if (!std::isfinite(config_.datasetRadiusMeters) ||
      config_.datasetRadiusMeters <= 0.0) {
    throw std::invalid_argument("datasetRadiusMeters must be positive");
  }
}

void Engine::initialize(std::vector<POI> pois) {
  pois_ = std::move(pois);
  frame_ = {};
  initialOrientation_ = {};
  initialHeadingDeg_ = 0.0;
  hasOrientationReference_ = false;
  frame_.projectedPOIs.reserve(pois_.size());
  frame_.visiblePOIs.reserve(
      std::min<std::size_t>(pois_.size(), config_.maxVisiblePOIs));
  initialized_ = true;
}

void Engine::setViewState(ViewState viewState) {
  if (!std::isfinite(viewState.horizontalFovDeg) ||
      viewState.horizontalFovDeg <= 1.0 ||
      viewState.horizontalFovDeg >= 179.0) {
    throw std::invalid_argument(
        "horizontalFovDeg must be between 1 and 179 degrees");
  }
  if (!std::isfinite(viewState.minDistanceMeters) ||
      !std::isfinite(viewState.maxDistanceMeters) ||
      viewState.minDistanceMeters < 0.0 ||
      viewState.maxDistanceMeters <= viewState.minDistanceMeters) {
    throw std::invalid_argument("invalid view-state distance range");
  }

  viewState_ = viewState;
}

std::uint64_t Engine::update(const SensorState& sensorState) {
  if (!initialized_) {
    throw std::logic_error("Cumquat engine must be initialized before update");
  }

  frame_.sequence += 1;
  frame_.timestampNs = sensorState.timestampNs;
  frame_.projectedPOIs.clear();
  frame_.visiblePOIs.clear();

  if (!hasOrientationReference_ &&
      sensorState.hasOrientationQuaternion &&
      sensorState.hasInitialHeading) {
    initialOrientation_ = sensorState.orientation;
    initialHeadingDeg_ = sensorState.initialHeadingDeg;
    hasOrientationReference_ = true;
  }

  SensorState projectionState = sensorState;
  if (sensorState.hasOrientationQuaternion) {
    if (hasOrientationReference_) {
      projectionState.orientation =
          projection::geographicallyAlignedOrientation(
              sensorState.orientation,
              initialOrientation_,
              initialHeadingDeg_);
    } else {
      // A DeviceMotion quaternion has no geographic north reference by itself.
      // Do not project it using an invented zero while awaiting the synchronized
      // startup heading.
      return frame_.sequence;
    }
  }

  for (std::uint32_t index = 0; index < pois_.size(); ++index) {
    const POI& poi = pois_[index];
    const Vec3 enu = geo::ecefToENU(geo::toECEF(poi.position), sensorState.location);
    const double distance = enu.length();

    if (distance > config_.datasetRadiusMeters) continue;

    VisiblePOI projected;
    projected.poiIndex = index;
    projected.distance = distance;
    projected.bearingDeg =
        geo::initialBearingDeg(sensorState.location, poi.position);
    projected.depth = distance;

    const Vec3 camera = projection::worldToCamera(enu, projectionState);
    double x = 0.0;
    double y = 0.0;
    double depth = distance;
    const bool insideViewport = projection::projectToScreen(
        camera, projectionState, viewState_, x, y, depth);

    projected.x = x;
    projected.y = y;
    projected.depth = depth;

    if (distance < viewState_.minDistanceMeters) {
      projected.clippedByDistance = DistanceClip::Min;
    } else if (distance > viewState_.maxDistanceMeters) {
      projected.clippedByDistance = DistanceClip::Max;
    } else {
      projected.visible = insideViewport;
      projected.clipped = !insideViewport;
    }

    frame_.projectedPOIs.push_back(projected);

    if (projected.visible &&
        frame_.visiblePOIs.size() < config_.maxVisiblePOIs) {
      frame_.visiblePOIs.push_back(projected);
    }
  }

  std::sort(
      frame_.visiblePOIs.begin(),
      frame_.visiblePOIs.end(),
      [](const VisiblePOI& left, const VisiblePOI& right) {
        return left.depth > right.depth;
      });

  return frame_.sequence;
}

const ViewState& Engine::getViewState() const noexcept {
  return viewState_;
}

const FrameSnapshot& Engine::getFrame() const noexcept {
  return frame_;
}

std::optional<PickResult> Engine::pick(
    double x,
    double y,
    double radiusPixels) const {
  const double radiusSquared = radiusPixels * radiusPixels;
  std::optional<PickResult> result;

  for (const VisiblePOI& poi : frame_.visiblePOIs) {
    const double dx = poi.x - x;
    const double dy = poi.y - y;
    const double distanceSquared = dx * dx + dy * dy;
    if (distanceSquared > radiusSquared) continue;

    const double distancePixels = std::sqrt(distanceSquared);
    if (!result || distancePixels < result->distancePixels) {
      result = PickResult{poi.poiIndex, distancePixels};
    }
  }

  return result;
}

bool Engine::isInitialized() const noexcept {
  return initialized_;
}

} // namespace cumquat
