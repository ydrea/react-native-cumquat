#pragma once

#include "../core/Types.h"
#include "../math/Vec3.h"

namespace cumquat::projection {

Vec3 worldToCamera(const Vec3& enu, const SensorState& sensorState);
bool projectToScreen(
    const Vec3& camera,
    const SensorState& sensorState,
    const ViewState& viewState,
    double& x,
    double& y,
    double& depth);

} // namespace cumquat::projection
