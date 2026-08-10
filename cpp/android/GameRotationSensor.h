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
   * Returns an Earth-from-device quaternion. During startup the Android
   * absolute rotation vector is paired with the game rotation vector; after
   * that frozen alignment, updates are driven only by the non-magnetic game
   * rotation vector.
   */
  bool latest(Quaternion& orientation) const;
  bool isAvailable() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace cumquat::android
