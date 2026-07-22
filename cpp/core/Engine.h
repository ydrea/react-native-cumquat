#pragma once

#include "Types.h"

#include <optional>
#include <vector>

namespace cumquat {

class Engine {
 public:
  explicit Engine(EngineConfig config = {});

  void initialize(std::vector<POI> pois);
  void setViewState(ViewState viewState);
  std::uint64_t update(const SensorState& sensorState);

  const ViewState& getViewState() const noexcept;
  const FrameSnapshot& getFrame() const noexcept;
  std::optional<PickResult> pick(double x, double y, double radiusPixels) const;
  bool isInitialized() const noexcept;

 private:
  EngineConfig config_;
  ViewState viewState_;
  std::vector<POI> pois_;
  FrameSnapshot frame_;
  bool initialized_{false};
};

} // namespace cumquat
