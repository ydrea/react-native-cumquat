/**
 * Converts a device-top compass heading into the rear-camera bearing for the
 * active screen rotation. Expo Location reports the former; AR projection
 * needs the latter.
 *
 * In a landscape app this adds the signed quarter-turn reported by
 * DeviceMotion.orientation. The returned bearing is normalized to [0, 360).
 */
export function cameraBearingFromDeviceHeading(
  deviceHeadingDegrees: number,
  screenOrientationDegrees: number
): number {
  if (!Number.isFinite(deviceHeadingDegrees)) return 0;

  const cameraBearing = deviceHeadingDegrees + screenOrientationDegrees;
  return ((cameraBearing % 360) + 360) % 360;
}
