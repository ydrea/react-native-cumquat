#pragma once

#include "../core/Types.h"
#include "../math/Vec3.h"

namespace cumquat::geo {

Vec3 toECEF(const GeoPoint& point);
Vec3 ecefToENU(const Vec3& ecef, const GeoPoint& origin);
double initialBearingDeg(const GeoPoint& from, const GeoPoint& to);

} // namespace cumquat::geo
