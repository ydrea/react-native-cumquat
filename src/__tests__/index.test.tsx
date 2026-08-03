import { expect, it, jest } from '@jest/globals';

jest.mock('react-native', () => ({
  TurboModuleRegistry: {
    getEnforcing: () => ({
      getVersion: () => 'test-cpp',
      createEngine: () => 1,
      initialize: () => undefined,
      setViewState: () => undefined,
      update: () => 7,
      getFrame: () => ({
        sequence: 7,
        timestampNs: 1,
        orientation: {
          quaternion: { x: 0, y: 0, z: 0, w: 1 },
          convention: 'earth-from-device',
        },
        projectedPOIs: [],
        visiblePOIs: [],
      }),
      pick: () => null,
      destroyEngine: () => undefined,
    }),
  },
}));

import { CumquatEngine, getCumquatNativeVersion } from '../index';

it('exposes the stateful native engine wrapper', () => {
  expect(getCumquatNativeVersion()).toBe('test-cpp');

  const engine = CumquatEngine.create({ datasetRadiusMeters: 1_000 });
  engine.initialize([]);
  engine.setViewState({
    horizontalFovDegrees: 90,
    minDistanceMeters: 0,
    maxDistanceMeters: 1_000,
  });

  expect(
    engine.update({
      timestampNs: 1,
      location: { latitude: 0, longitude: 0, altitude: 0 },
      headingDegrees: 0,
      pitchDegrees: 0,
      rollDegrees: 0,
      viewportWidth: 100,
      viewportHeight: 100,
    })
  ).toBe(7);
  expect(engine.getFrame()).toMatchObject({
    sequence: 7,
    orientation: {
      quaternion: { x: 0, y: 0, z: 0, w: 1 },
      convention: 'earth-from-device',
    },
  });
  expect(engine.pick(10, 10)).toBeNull();

  engine.dispose();
  expect(() => engine.getFrame()).toThrow('CumquatEngine has been disposed');
});
