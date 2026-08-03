import type { FrameOrientation, Quaternion } from 'react-native-cumquat';

export type Vec3 = readonly [x: number, y: number, z: number];

const EPSILON = 1e-9;

export function normalizeQuaternion(source: Quaternion): Quaternion {
  const length = Math.hypot(source.x, source.y, source.z, source.w);
  if (!Number.isFinite(length) || length < EPSILON) {
    return { x: 0, y: 0, z: 0, w: 1 };
  }

  return {
    x: source.x / length,
    y: source.y / length,
    z: source.z / length,
    w: source.w / length,
  };
}

export function inverseQuaternion(source: Quaternion): Quaternion {
  const q = normalizeQuaternion(source);
  return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

export function modelOrientation(frame: FrameOrientation): Quaternion {
  return frame.convention === 'earth-from-device'
    ? normalizeQuaternion(frame.quaternion)
    : inverseQuaternion(frame.quaternion);
}

export function rotateVector(source: Vec3, orientation: Quaternion): Vec3 {
  const q = normalizeQuaternion(orientation);
  const [x, y, z] = source;
  const tx = 2 * (q.y * z - q.z * y);
  const ty = 2 * (q.z * x - q.x * z);
  const tz = 2 * (q.x * y - q.y * x);

  return [
    x + q.w * tx + (q.y * tz - q.z * ty),
    y + q.w * ty + (q.z * tx - q.x * tz),
    z + q.w * tz + (q.x * ty - q.y * tx),
  ];
}

export function cameraBearing(frame: FrameOrientation | null): number | null {
  if (!frame) return null;

  const [east, north] = rotateVector([0, 0, -1], modelOrientation(frame));
  if (Math.hypot(east, north) < 0.01) return null;

  return ((Math.atan2(east, north) * 180) / Math.PI + 360) % 360;
}

export function slerp(
  fromSource: Quaternion,
  toSource: Quaternion,
  amount: number
): Quaternion {
  const from = normalizeQuaternion(fromSource);
  let to = normalizeQuaternion(toSource);
  let dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

  if (dot < 0) {
    dot = -dot;
    to = { x: -to.x, y: -to.y, z: -to.z, w: -to.w };
  }

  const t = Math.max(0, Math.min(1, amount));
  if (dot > 0.9995) {
    return normalizeQuaternion({
      x: from.x + t * (to.x - from.x),
      y: from.y + t * (to.y - from.y),
      z: from.z + t * (to.z - from.z),
      w: from.w + t * (to.w - from.w),
    });
  }

  const theta = Math.acos(Math.max(-1, Math.min(1, dot)));
  const denominator = Math.sin(theta);
  const fromWeight = Math.sin((1 - t) * theta) / denominator;
  const toWeight = Math.sin(t * theta) / denominator;

  return {
    x: from.x * fromWeight + to.x * toWeight,
    y: from.y * fromWeight + to.y * toWeight,
    z: from.z * fromWeight + to.z * toWeight,
    w: from.w * fromWeight + to.w * toWeight,
  };
}
