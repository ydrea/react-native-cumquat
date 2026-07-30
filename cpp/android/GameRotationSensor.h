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

  bool latest(Quaternion& orientation) const;
  bool isAvailable() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace cumquat::android
