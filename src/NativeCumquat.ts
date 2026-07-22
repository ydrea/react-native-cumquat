import type { TurboModule } from 'react-native';
import { TurboModuleRegistry } from 'react-native';

export interface Spec extends TurboModule {
  readonly getVersion: () => string;
  readonly createEngine: (config: Object) => number;
  readonly initialize: (handle: number, pois: ReadonlyArray<Object>) => void;
  readonly setViewState: (handle: number, viewState: Object) => void;
  readonly update: (handle: number, sensorState: Object) => number;
  readonly getFrame: (handle: number) => Object;
  readonly pick: (
    handle: number,
    x: number,
    y: number,
    radiusPixels: number
  ) => Object | null;
  readonly destroyEngine: (handle: number) => void;
}

export default TurboModuleRegistry.getEnforcing<Spec>('Cumquat');
