#pragma once

#include "../core/Types.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace cumquat::android {

struct TimedOrientation {
  Quaternion orientation;
  std::int64_t timestampNs{0};
};

/**
 * Establishes a constant transform from Android's arbitrary game-rotation
 * world into magnetic East/North/Up coordinates.
 *
 * The absolute rotation vector participates only while this object is
 * uncalibrated. Once alignment is accepted, alignGameOrientation() depends
 * solely on TYPE_GAME_ROTATION_VECTOR samples.
 */
class RotationAlignment {
 public:
  static constexpr std::size_t kRequiredSamples = 20;
  static constexpr std::int64_t kMaximumPairDeltaNs = 30'000'000;
  static constexpr double kMinimumFieldMicrotesla = 25.0;
  static constexpr double kMaximumFieldMicrotesla = 65.0;
  static constexpr double kMaximumCandidateErrorDeg = 5.0;

  void reset();
  bool addPair(
      const TimedOrientation& absolute,
      const TimedOrientation& game,
      std::optional<double> magneticFieldMicrotesla = std::nullopt);

  bool isCalibrated() const noexcept;
  std::size_t acceptedSampleCount() const noexcept;
  const Quaternion& alignment() const noexcept;
  Quaternion alignGameOrientation(const Quaternion& gameOrientation) const;

 private:
  std::vector<Quaternion> candidates_;
  Quaternion alignment_;
  bool calibrated_{false};
};

Quaternion normalizedQuaternion(const Quaternion& source);
Quaternion multiplyQuaternions(
    const Quaternion& left,
    const Quaternion& right);
Quaternion inverseQuaternion(const Quaternion& source);
double quaternionAngularDistanceDeg(
    const Quaternion& left,
    const Quaternion& right);

} // namespace cumquat::android
