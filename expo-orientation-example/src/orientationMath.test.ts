import assert from 'node:assert/strict';
import test from 'node:test';

import {
  inverseQuaternion,
  modelOrientation,
  normalizeQuaternion,
  rotateVector,
  slerp,
} from './orientationMath';

const EPSILON = 1e-9;

function near(actual: number, expected: number) {
  assert.ok(Math.abs(actual - expected) < EPSILON, `${actual} != ${expected}`);
}

test('normalizes a quaternion', () => {
  const q = normalizeQuaternion({ x: 0, y: 0, z: 2, w: 2 });
  near(Math.hypot(q.x, q.y, q.z, q.w), 1);
});

test('rotates device X onto world Y for a 90 degree Z rotation', () => {
  const half = Math.SQRT1_2;
  const [x, y, z] = rotateVector([1, 0, 0], { x: 0, y: 0, z: half, w: half });
  near(x, 0);
  near(y, 1);
  near(z, 0);
});

test('inverts world-to-camera for the displayed model', () => {
  const source = { x: 0.1, y: -0.2, z: 0.3, w: 0.9 };
  assert.deepEqual(
    modelOrientation({ quaternion: source, convention: 'world-to-camera' }),
    inverseQuaternion(source)
  );
});

test('slerp chooses the short quaternion arc', () => {
  const identity = { x: 0, y: 0, z: 0, w: 1 };
  const sameRotation = { x: 0, y: 0, z: 0, w: -1 };
  assert.deepEqual(slerp(identity, sameRotation, 0.5), identity);
});
