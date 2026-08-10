import NativeCumquat from './NativeCumquat';

export { CumquatEngine } from './CumquatEngine';
export { cameraBearingFromDeviceHeading } from './orientation';

export function getCumquatNativeVersion(): string {
  return NativeCumquat.getVersion();
}

export type {
  EngineConfig,
  FrameOrientation,
  FrameSnapshot,
  GeoPoint,
  PickResult,
  POIInput,
  ProjectedPOI,
  Quaternion,
  SensorState,
  ViewState,
  VisiblePOI,
} from './types';
