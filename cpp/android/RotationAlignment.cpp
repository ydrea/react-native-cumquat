#include "RotationAlignment.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../math/Vec3.h"

namespace cumquat::android {
namespace {
constexpr double kRadiansToDegrees =
    180.0 / 3.14159265358979323846;

double dot(const Quaternion& left, const Quaternion& right) {
  return left.x * right.x +
      left.y * right.y +
      left.z * right.z +
      left.w * right.w;
}

Quaternion negated(const Quaternion& source) {
  return {-source.x, -source.y, -source.z, -source.w};
}

Quaternion averageCandidates(const std::vector<Quaternion>& candidates) {
  if (candidates.empty()) return {};

  const Quaternion reference = candidates.front();
  Quaternion sum{};
  sum.w = 0.0;

  for (const Quaternion& candidate : candidates) {
    const Quaternion aligned =
        dot(reference, candidate) < 0.0 ? negated(candidate) : candidate;
    sum.x += aligned.x;
    sum.y += aligned.y;
    sum.z += aligned.z;
    sum.w += aligned.w;
  }

  return normalizedQuaternion(sum);
}

Vec3 rotateVector(
    const Quaternion& source,
    const Vec3& vector) {
  const Quaternion q = normalizedQuaternion(source);
  const double tx = 2.0 * (q.y * vector.z - q.z * vector.y);
  const double ty = 2.0 * (q.z * vector.x - q.x * vector.z);
  const double tz = 2.0 * (q.x * vector.y - q.y * vector.x);
  return {
      vector.x + q.w * tx + (q.y * tz - q.z * ty),
      vector.y + q.w * ty + (q.z * tx - q.x * tz),
      vector.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}
} // namespace

Quaternion normalizedQuaternion(const Quaternion& source) {
  const double norm = std::sqrt(
      source.x * source.x +
      source.y * source.y +
      source.z * source.z +
      source.w * source.w);
  if (!std::isfinite(norm) ||
      norm <= std::numeric_limits<double>::epsilon()) {
    return {};
  }

  const double scale = 1.0 / norm;
  return {
      source.x * scale,
      source.y * scale,
      source.z * scale,
      source.w * scale,
  };
}

Quaternion multiplyQuaternions(
    const Quaternion& left,
    const Quaternion& right) {
  return normalizedQuaternion({
      left.w * right.x + left.x * right.w +
          left.y * right.z - left.z * right.y,
      left.w * right.y - left.x * right.z +
          left.y * right.w + left.z * right.x,
      left.w * right.z + left.x * right.y -
          left.y * right.x + left.z * right.w,
      left.w * right.w - left.x * right.x -
          left.y * right.y - left.z * right.z,
  });
}

Quaternion inverseQuaternion(const Quaternion& source) {
  const Quaternion normalized = normalizedQuaternion(source);
  return {
      -normalized.x,
      -normalized.y,
      -normalized.z,
      normalized.w,
  };
}

double quaternionAngularDistanceDeg(
    const Quaternion& left,
    const Quaternion& right) {
  const double cosine = std::clamp(
      std::abs(dot(
          normalizedQuaternion(left),
          normalizedQuaternion(right))),
      0.0,
      1.0);
  return 2.0 * std::acos(cosine) * kRadiansToDegrees;
}

void RotationAlignment::reset() {
  candidates_.clear();
  alignment_ = {};
  calibrated_ = false;
}

bool RotationAlignment::addPair(
    const TimedOrientation& absolute,
    const TimedOrientation& game,
    std::optional<double> magneticFieldMicrotesla) {
  if (calibrated_) return true;

  const std::int64_t pairDelta =
      absolute.timestampNs >= game.timestampNs
      ? absolute.timestampNs - game.timestampNs
      : game.timestampNs - absolute.timestampNs;
  if (pairDelta > kMaximumPairDeltaNs) return false;

  if (magneticFieldMicrotesla.has_value()) {
    const double field = *magneticFieldMicrotesla;
    if (!std::isfinite(field) ||
        field < kMinimumFieldMicrotesla ||
        field > kMaximumFieldMicrotesla) {
      candidates_.clear();
      return false;
    }
  }

  // Android +Z points out through the display, so the rear camera looks -Z.
  // Its Earth-frame horizontal projection must be long enough to define a
  // geographic bearing; reject near-vertical camera poses.
  const Vec3 forwardEarth = rotateVector(
      absolute.orientation,
      {0.0, 0.0, -1.0});
  if (std::hypot(forwardEarth.x, forwardEarth.y) < 0.15) {
    candidates_.clear();
    return false;
  }

  const Quaternion candidate = multiplyQuaternions(
      normalizedQuaternion(absolute.orientation),
      inverseQuaternion(game.orientation));
  candidates_.push_back(candidate);

  if (candidates_.size() < kRequiredSamples) return false;

  const Quaternion average = averageCandidates(candidates_);
  for (const Quaternion& sample : candidates_) {
    if (quaternionAngularDistanceDeg(sample, average) >
        kMaximumCandidateErrorDeg) {
      // Keep a recent window so one disturbed sample cannot permanently poison
      // calibration, while still requiring a complete stable window.
      candidates_.erase(candidates_.begin());
      return false;
    }
  }

  alignment_ = average;
  calibrated_ = true;
  candidates_.clear();
  return true;
}

bool RotationAlignment::isCalibrated() const noexcept {
  return calibrated_;
}

std::size_t RotationAlignment::acceptedSampleCount() const noexcept {
  return candidates_.size();
}

const Quaternion& RotationAlignment::alignment() const noexcept {
  return alignment_;
}

Quaternion RotationAlignment::alignGameOrientation(
    const Quaternion& gameOrientation) const {
  if (!calibrated_) return {};
  return multiplyQuaternions(
      alignment_,
      normalizedQuaternion(gameOrientation));
}

} // namespace cumquat::android
