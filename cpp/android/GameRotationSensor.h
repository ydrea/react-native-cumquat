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
   * Returns the current device-to-magnetic-ENU orientation. Before startup
   * calibration completes this returns false; after calibration it is driven
   * only by TYPE_GAME_ROTATION_VECTOR.
   */
  bool latest(Quaternion& orientation) const;
  bool isAvailable() const;
  bool isCalibrated() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace cumquat::android
