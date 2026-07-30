#pragma once

#include <CumquatSpecJSI.h>
#include "../android/GameRotationSensor.h"

#include "../core/Engine.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace facebook::react {

class NativeCumquatModule final
    : public NativeCumquatCxxSpec<NativeCumquatModule> {
 public:
  explicit NativeCumquatModule(std::shared_ptr<CallInvoker> jsInvoker);
  ~NativeCumquatModule() override;

  std::string getVersion(jsi::Runtime& runtime);
  double createEngine(jsi::Runtime& runtime, jsi::Object config);
  void initialize(jsi::Runtime& runtime, double handle, jsi::Array pois);
  void setViewState(
      jsi::Runtime& runtime,
      double handle,
      jsi::Object viewState);
  double update(jsi::Runtime& runtime, double handle, jsi::Object sensorState);
  jsi::Object getFrame(jsi::Runtime& runtime, double handle);
  std::optional<jsi::Object> pick(
      jsi::Runtime& runtime,
      double handle,
      double x,
      double y,
      double radiusPixels);
  void destroyEngine(jsi::Runtime& runtime, double handle);

 private:
  cumquat::android::GameRotationSensor gameRotationSensor_;
  using EnginePtr = std::shared_ptr<cumquat::Engine>;

  EnginePtr requireEngine(jsi::Runtime& runtime, double handle) const;
  static std::uint64_t validateHandle(jsi::Runtime& runtime, double handle);

  mutable std::mutex mutex_;
  std::unordered_map<std::uint64_t, EnginePtr> engines_;
  std::uint64_t nextHandle_{1};
};

} // namespace facebook::react
