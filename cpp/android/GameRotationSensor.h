#pragma once

#include "../core/Types.h"

#include <memory>

namespace cumquat::android {

class GameRotationSensor {
 public:
  GameRotationSensor();
  ~GameRotationSensor();

  GameRotationSensor(const GameRotationSensor&) = delete;
  GameRotationSensor& operator=(const GameRotationSensor&) = delete;

  /**
   * Returns Android's current TYPE_GAME_ROTATION_VECTOR quaternion. This
   * sensor deliberately has no geographic north; Engine pairs its first usable
   * sample with initialHeadingDegrees and freezes that reference.
   */
  bool latest(Quaternion& orientation) const;
  bool isAvailable() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace cumquat::android
