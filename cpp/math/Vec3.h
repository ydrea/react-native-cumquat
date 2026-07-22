#pragma once

#include <cmath>

namespace cumquat {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 operator+(const Vec3& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }

  Vec3 operator-(const Vec3& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }

  Vec3 operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  double length() const {
    return std::sqrt(x * x + y * y + z * z);
  }
};

} // namespace cumquat
