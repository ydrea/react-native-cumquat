export type EngineHandle = number;

export type GeoPoint = {
  latitude: number;
  longitude: number;
  altitude: number;
};

export type Quaternion = {
  x: number;
  y: number;
  z: number;
  w: number;
};

export type POIInput = GeoPoint & {
  id: string;
  name?: string;
};

export type EngineConfig = {
  datasetRadiusMeters?: number;
  maxVisiblePOIs?: number;
};

export type ViewState = {
  horizontalFovDegrees: number;
  minDistanceMeters: number;
  maxDistanceMeters: number;
};

export type SensorState = {
  timestampNs: number;
  location: GeoPoint;
  orientationQuaternion?: Quaternion;
  /** Geographic heading paired with the first orientation quaternion.
   * Consumed once per initialized engine; later values are ignored. */
  initialHeadingDegrees?: number;
  headingDegrees: number;
  pitchDegrees: number;
  rollDegrees: number;
  viewportWidth: number;
  viewportHeight: number;
};

export type VisiblePOI = {
  poiIndex: number;
  x: number;
  y: number;
  depth: number;
  distance: number;
  bearing: number;
  visible: boolean;
};

export type ProjectedPOI = VisiblePOI & {
  clipped: boolean;
  clippedByDistance: 'min' | 'max' | null;
};

/** The exact normalized quaternion used by Cumquat for the latest frame. */
export type FrameOrientation = {
  quaternion: Quaternion;
  /**
   * `earth-from-device` rotates physical device axes into geographic ENU.
   * `world-to-camera` rotates geographic ENU into Cumquat's camera axes.
   */
  convention: 'earth-from-device' | 'world-to-camera';
};

export type FrameSnapshot = {
  sequence: number;
  timestampNs: number;
  /** Null until the native sensor or synchronized orientation reference is ready. */
  orientation: FrameOrientation | null;
  projectedPOIs: readonly ProjectedPOI[];
  visiblePOIs: readonly ProjectedPOI[];
};

export type PickResult = {
  poiIndex: number;
  distancePixels: number;
} | null;
